// Copyright (c) 2021, Benjamin Rosenbach
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#define HEM_SHREDDER_ANIMATION_SPEED 500
#define HEM_SHREDDER_DOUBLE_CLICK_DELAY 5000
#define HEM_SHREDDER_POS_5V 7680 // 5 * (12 << 7)
#define HEM_SHREDDER_NEG_3V 4608 // 3 * (12 << 7)

class Shredder : public HemisphereApplet {
public:
    enum ShredCursor {
      CHAN1_RANGE, CHAN2_RANGE,
      QUANT_CHAN,
      QUANT_EDIT,
      RESETSHRED1, RESETSHRED2,

      MAX_CURSOR = RESETSHRED2
    };

    const char* applet_name() {
        return "Shredder";
    }
    const uint8_t* applet_icon() { return PhzIcons::shredder; }

    void Start() {
        step = 0;
        replay = 0;
        reset = true;
        quant_channels = 0;
        ForEachChannel(ch) {
            Shred(ch);
        }
        VoltageOut();
    }

    void Reset() {
        step = 0; // Reset
        reset = true;
        VoltageOut();
        ForEachChannel(ch) {
          if (shred_on_reset[ch])
            Shred(ch);
        }
    }

    void Controller() {
        if (Clock(1)) {
            Reset();
        }

        if (Clock(0)) {
            // Are the X or Y position being set? If so, get step coordinates. Otherwise,
            // simply play current step and advance it. This way, the applet can be used as
            // a more conventional arpeggiator as well as a Cartesian one.
            if (DetentedIn(0) || DetentedIn(1)) {
                int x = ProportionCV(In(0), 4);
                int y = ProportionCV(In(1), 4);
                if (x > 3) x = 3;
                if (y > 3) y = 3;
                step = (y * 4) + x;
                VoltageOut();
            } else {
                if (!reset) {
                    ++step;
                }
                reset = false;
                if (step > 15) step = 0;
                VoltageOut();
            }
            replay = 0;
        } else if (replay) {
            VoltageOut();
            replay = 0;
        }

        // Handle imprint confirmation animation
        if (--confirm_animation_countdown < 0) {
            confirm_animation_position--;
            confirm_animation_countdown = HEM_SHREDDER_ANIMATION_SPEED;
        }
    }

    void View() {
        DrawParams();
        DrawMeters();
        DrawGrid();
    }

    void AuxButton() {
      if (cursor < 2) {
        Shred(cursor);
      }
      else
        isEditing = false;
    }

    void OnButtonPress() {
      switch (cursor) {
        case CHAN1_RANGE:
        case CHAN2_RANGE:
          if (OC::CORE::ticks - click_tick < HS::HEMISPHERE_DOUBLE_CLICK_TIME)
            Shred(cursor);
          else
            click_tick = OC::CORE::ticks;
        default:
          CursorToggle();
          break;

        case QUANT_EDIT:
          HS::QuantizerEdit(io_offset);
          break;

        case RESETSHRED1:
        case RESETSHRED2:
          shred_on_reset[cursor - RESETSHRED1] = !shred_on_reset[cursor - RESETSHRED1];
          break;
      }
    }

    void OnEncoderMove(int direction) {
        if (!EditMode()) {
            MoveCursor(cursor, direction, MAX_CURSOR);
            return;
        }

        if (cursor < 2) {
            range[cursor] += direction;
            if (bipolar[cursor]) {
                if (range[cursor] > 3) {
                    range[cursor] = 0;
                    bipolar[cursor] = false;
                } else if (range[cursor] < 1) {
                    range[cursor] = 5;
                    bipolar[cursor] = false;
                }
            } else {
                if (range[cursor] > 5) {
                    range[cursor] = 1;
                    bipolar[cursor] = true;
                } else if (range[cursor] < 0) {
                    range[cursor] = 3;
                    bipolar[cursor] = true;
                }
            }
        }
        if (QUANT_CHAN == cursor)
          quant_channels = constrain(quant_channels + direction, 0, 2);
    }

    uint64_t OnDataRequest() {
        uint64_t data = 0;
        // Not enough room to save the sequences, so we'll just have to save settings
        Pack(data, PackLocation {0,4}, range[0]); // range will never be more than 4 bits
        Pack(data, PackLocation {4,1}, int(bipolar[0]));
        Pack(data, PackLocation {5,1}, shred_on_reset[0]);
        Pack(data, PackLocation {8,4}, range[1]);
        Pack(data, PackLocation {12,1}, int(bipolar[1]));
        Pack(data, PackLocation {13,1}, shred_on_reset[1]);
        Pack(data, PackLocation {16,8}, quant_channels);
        Pack(data, PackLocation {24,8}, GetScale(0));
        return data;
    }

    void OnDataReceive(uint64_t data) {
        range[0] = Unpack(data, PackLocation {0,4}); // only 4 bits used for range
        bipolar[0] = Unpack(data, PackLocation {4,1}); 
        shred_on_reset[0] = Unpack(data, PackLocation {5,1});
        range[1] = Unpack(data, PackLocation {8,4});
        bipolar[1] = Unpack(data, PackLocation {12,1}); 
        shred_on_reset[1] = Unpack(data, PackLocation {13,1});
        quant_channels = Unpack(data, PackLocation {16,8});
        SetScale(0, Unpack(data, PackLocation {24,8}));
        ForEachChannel(ch) {
            Shred(ch);
        }
        VoltageOut();
    }

protected:
    void SetHelp() {
        //                    "-------" <-- Label size guide
        help[HELP_DIGITAL1] = "Clock";
        help[HELP_DIGITAL2] = "Reset";
        help[HELP_CV1]      = "X pos";
        help[HELP_CV2]      = "Y pos";
        help[HELP_OUT1]     = "Ch 1";
        help[HELP_OUT2]     = "Ch 2";
        help[HELP_EXTRA1] = "";
        help[HELP_EXTRA2] = "AuxButton to Shred";
        //                  "---------------------" <-- Extra text size guide
    }

private:
    int cursor;
    uint32_t click_tick = 0;

    // Sequencer state
    uint8_t step; // Current step number
    int sequence[2][16];
    int current[2];
    bool replay; // When the encoder is moved, re-quantize the output
    bool reset;

    // settings
    int range[2] = {1,0};
    bool bipolar[2] = {false, false};
    bool shred_on_reset[2] = {false, false};
    int8_t quant_channels;

    // Variables to handle imprint confirmation animation
    int confirm_animation_countdown;
    int confirm_animation_position;

    void DrawParams() {
        // Channel 1 voltage
        char outlabel[] = { (char)('A' + io_offset), ':', '+', '\0' };
        gfxPrint(1, 15, outlabel);
        if (shred_on_reset[0]) gfxInvert(1, 15, 12, 8);
        gfxPrint(19, 15, (char) (range[0]));
        if (bipolar[0]) {
          gfxPrint(13, 18, "-");
        }
        if (CHAN1_RANGE == cursor) gfxSpicyCursor(13, 23, 12);

        // Channel 2 voltage
        ++outlabel[0];
        gfxPrint(33, 15, outlabel);
        if (shred_on_reset[1]) gfxInvert(33, 15, 12, 8);
        gfxPrint(51, 15, (char) (range[1]));
        if (bipolar[1]) {
          gfxPrint(45, 18, "-");
        }
        if (CHAN2_RANGE == cursor) gfxSpicyCursor(45, 23, 12);

        if (cursor >= RESETSHRED1) {
          gfxIcon(1 + 32*(cursor-RESETSHRED1), 8, DOWN_BTN_ICON);
        }

        // quantize channel selection
        gfxIcon(32, 25, SCALE_ICON);
        if (quant_channels == 0) {
          outlabel[0] = 'A'+io_offset;
          outlabel[1] = '+';
          outlabel[2] = 'B'+io_offset;
          gfxPrint(42, 25, outlabel);
        } else {
          gfxPrint(48, 25, OutputLabel(quant_channels - 1));
        }
        if (QUANT_CHAN == cursor) gfxCursor(42, 33, 20);

        // quantize scale selection
        gfxPrint(32, 35, OC::scale_names_short[GetScale(0)]);
        if (QUANT_EDIT == cursor) gfxCursor(32, 43, 30);

    }

    void DrawMeters() {
      ForEachChannel(ch) {
        int o = ch * 10; // offset
        gfxLine(34, 47+o, 62, 47+o); // top line
        gfxLine(34, 50+o, 62, 50+o); // bottom line
        gfxLine(44, 45+o, 44, 52+o); // zero line
        // 10 pixels for neg, 18 pixels for pos
        if (current[ch] > 0) {
          int w = Proportion(current[ch], HEM_SHREDDER_POS_5V, 18);
          gfxRect(45, 48+o, w, 2);
        } else {
          int w = Proportion(-current[ch], HEM_SHREDDER_NEG_3V, 10);
          gfxRect(44-w, 48+o, w, 2);
        }
      }
    }
    
    void DrawGrid() {
        // Draw the Cartesian plane
        for (int s = 0; s < 16; s++) gfxFrame(1 + (8 * (s % 4)), 26 + (8 * (s / 4)), 5, 5);

        // Crosshairs for play position
        int cxy = step / 4;
        int cxx = step % 4;
        gfxDottedLine(3 + (8 * cxx), 26, 3 + (8 * cxx), 58, 2);
        gfxDottedLine(1, 28 + (8 * cxy), 32, 28 + (8 * cxy), 2);
        gfxRect(1 + (8 * cxx), 26 + (8 * cxy), 5, 5);

        // Draw imprint animation, if necessary
        if (confirm_animation_position > -1) {
            int progress = 16 - confirm_animation_position;
            for (int s = 0; s < progress; s++)
            {
                gfxRect(1 + (8 * (s / 4)), 26 + (8 * (s % 4)), 7, 7);
            }
        }
    }

    void Shred(int ch) {
        int max;
        int min;
        for (int i = 0; i < 16; i++) {
            if (range[ch] == 0) {
                sequence[ch][i] = 0;
            } else {
                max = range[ch] * (12 << 7);
                min = bipolar[ch] ? -max : 0;
                sequence[ch][i] = random(min, max);
            }
        }

        // start imprint animation
        confirm_animation_position = 16;
        confirm_animation_countdown = HEM_SHREDDER_ANIMATION_SPEED;
    }

    void VoltageOut() {
        ForEachChannel(ch) {
            current[ch] = sequence[ch][step];
            int8_t qc = quant_channels - 1; 
            if (qc < 0 || qc == ch) current[ch] = Quantize(0, current[ch]);
            Out(ch, current[ch]);
        }
    }
};
