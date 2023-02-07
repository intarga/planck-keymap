/* Copyright 2015-2021 Jack Humbert
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include QMK_KEYBOARD_H
#include "muse.h"


enum planck_layers {
  _COLEMAK,
  _QWERTY,
  _LOWER,
  _RAISE,
  _ADJUST
};

enum planck_keycodes {
  QWERTY = SAFE_RANGE,
  COLEMAK,
  FERRIS,
  PLOVER,
  BACKLIT,
  EXT_PLV
};

// Left-hand home row mods
#define HOME_R LCTL_T(KC_R)
#define HOME_S LALT_T(KC_S)
#define HOME_T LGUI_T(KC_T)

// Right-hand home row mods
#define HOME_N RGUI_T(KC_N)
#define HOME_E LALT_T(KC_E)
#define HOME_I RCTL_T(KC_I)

// Norwegian letters
#define NO_AE LALT(KC_QUOT)
#define NO_AA LALT(KC_A)
#define NO_OE LALT(KC_O)

// Ctrl U and D
#define CTRL_U LCTL(KC_U)
#define CTRL_D LCTL(KC_D)

// Layer keys
#define LOWER MO(_LOWER)
#define RAISE MO(_RAISE)

// #define L1 TO(_FERRIS)
// #define L2 MO(_L2)
// #define L3 MO(_L3)
#define TO_COLE TO(_COLEMAK)

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
[_COLEMAK] = LAYOUT_planck_grid(
    KC_LCTL, KC_Q,    KC_W,    KC_F,    KC_P,    KC_G,    KC_J,    KC_L,    KC_U,    KC_Y,    KC_QUOT, KC_LALT,
    KC_ESC,  KC_A,    HOME_R,  HOME_S,  HOME_T,  KC_D,    KC_H,    HOME_N,  HOME_E,  HOME_I,  KC_O,    KC_BSPC,
    KC_TAB,  KC_Z,    KC_X,    KC_C,    KC_V,    KC_B,    KC_K,    KC_M,    KC_COMM, KC_DOT,  KC_SLSH, KC_ENT ,
    _______, _______, _______, KC_LGUI, RAISE,   KC_SPC,  KC_SPC,  LOWER,   KC_LSFT, _______, _______, _______
),
[_QWERTY] = LAYOUT_planck_grid(
    KC_LCTL, KC_Q,    KC_W,    KC_E,    KC_R,    KC_T,    KC_Y,    KC_U,    KC_I,    KC_O,    KC_P,    KC_QUOT,
    KC_ESC,  KC_A,    KC_S,    KC_D,    KC_F,    KC_G,    KC_H,    KC_J,    KC_K,    KC_L,    KC_SCLN, KC_BSPC,
    KC_TAB,  KC_Z,    KC_X,    KC_C,    KC_V,    KC_B,    KC_N,    KC_M,    KC_COMM, KC_DOT,  KC_SLSH, KC_ENT ,
    _______, _______, _______, KC_LGUI, RAISE,   KC_SPC,  KC_SPC,  LOWER,   KC_LSFT, _______, _______, _______
),
[_LOWER] = LAYOUT_planck_grid(
    _______, _______, _______, KC_SCLN, KC_GRV,  KC_LPRN, KC_RPRN, CTRL_D,  CTRL_U,  _______, _______, _______,
    _______, KC_DEL,  KC_ESC,  KC_TAB,  KC_ENT,  KC_SCLN, KC_LEFT, KC_DOWN, KC_UP,   KC_RGHT, _______, _______,
    _______, _______, _______, KC_BSLS, KC_TILD, KC_LBRC, KC_RBRC, KC_PGDN, KC_PGUP, _______, _______, _______,
    _______, _______, _______, _______, _______, KC_BSPC, KC_BSPC, _______, _______, _______, _______, _______
),
[_RAISE] = LAYOUT_planck_grid(
    _______, _______, KC_MUTE, KC_VOLD, KC_VOLU, KC_BRIU, KC_0,    KC_1,    KC_2,    KC_3,    KC_BSPC, _______,
    _______, _______, NO_AE,   NO_OE,   NO_AA,   KC_BRID, KC_MINS, KC_4,    KC_5,    KC_6,    KC_PLUS, _______,
    _______, _______, KC_MRWD, KC_MPLY, KC_MFFD, _______, KC_EQL,  KC_7,    KC_8,    KC_9,    KC_ENT,  _______,
    _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______, _______
),
[_ADJUST] = LAYOUT_planck_grid(
    QK_BOOT, KC_F1,   KC_F2,   KC_F3,   KC_F4,   MU_MOD,  RGB_HUI, RGB_HUD, RGB_SAI, RGB_SAD, RGB_VAI, RGB_VAD,
    COLEMAK, KC_F5,   KC_F6,   KC_F7,   KC_F8,   MUV_DE,  RGB_TOG, RGB_MOD, AU_ON,   AU_OFF,  AG_NORM, AG_SWAP,
    QWERTY,  KC_F9,   KC_F10,  KC_F11,  KC_F12,  MUV_IN,  MU_ON,   MU_OFF,  MI_ON,   MI_OFF,  TERM_ON, TERM_OFF,
    DEBUG,   _______, _______, _______, _______, BACKLIT, BACKLIT, _______, _______,  _______, _______, _______
)
};

#ifdef AUDIO_ENABLE
  float plover_song[][2]     = SONG(PLOVER_SOUND);
  float plover_gb_song[][2]  = SONG(PLOVER_GOODBYE_SOUND);
#endif

layer_state_t layer_state_set_user(layer_state_t state) {
  return update_tri_layer_state(state, _LOWER, _RAISE, _ADJUST);
}

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
  switch (keycode) {
    case QWERTY:
      if (record->event.pressed) {
        print("mode just switched to qwerty and this is a huge string\n");
        set_single_persistent_default_layer(_QWERTY);
      }
      return false;
      break;
    case COLEMAK:
      if (record->event.pressed) {
        set_single_persistent_default_layer(_COLEMAK);
      }
      return false;
      break;
    // case FERRIS:
    //   if (record->event.pressed) {
    //     set_single_persistent_default_layer(_FERRIS);
    //   }
    //   return false;
    //   break;
    case BACKLIT:
      if (record->event.pressed) {
        register_code(KC_RSFT);
        #ifdef BACKLIGHT_ENABLE
          backlight_step();
        #endif
        #ifdef KEYBOARD_planck_rev5
          writePinLow(E6);
        #endif
      } else {
        unregister_code(KC_RSFT);
        #ifdef KEYBOARD_planck_rev5
          writePinHigh(E6);
        #endif
      }
      return false;
      break;
  }
  return true;
}

bool muse_mode = false;
uint8_t last_muse_note = 0;
uint16_t muse_counter = 0;
uint8_t muse_offset = 70;
uint16_t muse_tempo = 50;

bool encoder_update_user(uint8_t index, bool clockwise) {
  if (muse_mode) {
    if (IS_LAYER_ON(_RAISE)) {
      if (clockwise) {
        muse_offset++;
      } else {
        muse_offset--;
      }
    } else {
      if (clockwise) {
        muse_tempo+=1;
      } else {
        muse_tempo-=1;
      }
    }
  } else {
    if (clockwise) {
      #ifdef MOUSEKEY_ENABLE
        tap_code(KC_MS_WH_DOWN);
      #else
        tap_code(KC_PGDN);
      #endif
    } else {
      #ifdef MOUSEKEY_ENABLE
        tap_code(KC_MS_WH_UP);
      #else
        tap_code(KC_PGUP);
      #endif
    }
  }
    return true;
}

bool dip_switch_update_user(uint8_t index, bool active) {
    switch (index) {
        case 0: {
#ifdef AUDIO_ENABLE
            static bool play_sound = false;
#endif
            if (active) {
#ifdef AUDIO_ENABLE
                if (play_sound) { PLAY_SONG(plover_song); }
#endif
                layer_on(_ADJUST);
            } else {
#ifdef AUDIO_ENABLE
                if (play_sound) { PLAY_SONG(plover_gb_song); }
#endif
                layer_off(_ADJUST);
            }
#ifdef AUDIO_ENABLE
            play_sound = true;
#endif
            break;
        }
        case 1:
            if (active) {
                muse_mode = true;
            } else {
                muse_mode = false;
            }
    }
    return true;
}

void matrix_scan_user(void) {
#ifdef AUDIO_ENABLE
    if (muse_mode) {
        if (muse_counter == 0) {
            uint8_t muse_note = muse_offset + SCALE[muse_clock_pulse()];
            if (muse_note != last_muse_note) {
                stop_note(compute_freq_for_midi_note(last_muse_note));
                play_note(compute_freq_for_midi_note(muse_note), 0xF);
                last_muse_note = muse_note;
            }
        }
        muse_counter = (muse_counter + 1) % muse_tempo;
    } else {
        if (muse_counter) {
            stop_all_notes();
            muse_counter = 0;
        }
    }
#endif
}

bool music_mask_user(uint16_t keycode) {
  switch (keycode) {
    case RAISE:
    case LOWER:
      return false;
    default:
      return true;
  }
}
