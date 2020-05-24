#include QMK_KEYBOARD_H
#include "version.h"
#include "keymap_german.h"
#include "keymap_nordic.h"
#include "keymap_french.h"
#include "keymap_spanish.h"
#include "keymap_hungarian.h"
#include "keymap_swedish.h"
#include "keymap_br_abnt2.h"
#include "keymap_canadian_multilingual.h"
#include "keymap_german_ch.h"
#include "keymap_jp.h"
#include "keymap_bepo.h"
#include "keymap_italian.h"
#include "keymap_slovenian.h"
#include "keymap_danish.h"
#include "keymap_norwegian.h"
#include "keymap_portuguese.h"

#define KC_MAC_UNDO LGUI(KC_Z)
#define KC_MAC_CUT LGUI(KC_X)
#define KC_MAC_COPY LGUI(KC_C)
#define KC_MAC_PASTE LGUI(KC_V)
#define KC_PC_UNDO LCTL(KC_Z)
#define KC_PC_CUT LCTL(KC_X)
#define KC_PC_COPY LCTL(KC_C)
#define KC_PC_PASTE LCTL(KC_V)
#define ES_LESS_MAC KC_GRAVE
#define ES_GRTR_MAC LSFT(KC_GRAVE)
#define ES_BSLS_MAC ALGR(KC_6)
#define NO_PIPE_ALT KC_GRAVE
#define NO_BSLS_ALT KC_EQUAL
#define LSA_T(kc) MT(MOD_LSFT | MOD_LALT, kc)


//=============  spatdsg - tap dance press and hold
//============================= semicolon
//single press will do a semicolon
//press and hold - will do a colon

//============================= arrows  no i tried this and it was dumb
//press and hold - does normal repeat pattern
//double tap - ctrl+arrow will jump blocks of text
//double tap hold - ctrl+shift+arrow will select blocks of text
//============================= HOME AND END
//press and hold - will *select all* to home or all to end
// double tap will jump to home or end of document



typedef struct {
  bool is_press_action;
  int state;
} tap;

enum {
  SINGLE_TAP = 1,
  SINGLE_HOLD = 2,
  DOUBLE_TAP = 3,
  DOUBLE_HOLD = 4,
  DOUBLE_SINGLE_TAP = 5, //send two single taps
  TRIPLE_TAP = 6,
  TRIPLE_HOLD = 7
};

//Tap dance enums
//see https://jayliu50.github.io/qmk-cheatsheet/
// I use X_(KEY) as in PRESS key.. so "PRESS LEFT ARROW"
enum {
  X_CTL = 0,
  SEMICOLON_PRESSANDHOLD_COLON = 1,
  X_LFT_ARROW = 2,
  X_RT_ARROW = 3,
  X_KC_HOME = 4,
  X_KC_END = 5

};

int cur_dance (qk_tap_dance_state_t *state);

//for the x tap dance. Put it here so it can be used in any keymap
//_finshed
//_reset
void x_finished (qk_tap_dance_state_t *state, void *user_data);
void x_reset (qk_tap_dance_state_t *state, void *user_data);
void SEMICOLON_PRESSANDHOLD_COLON_finished (qk_tap_dance_state_t *state, void *user_data);
void SEMICOLON_PRESSANDHOLD_COLON_reset (qk_tap_dance_state_t *state, void *user_data);

void X_LFT_ARROW_finished(qk_tap_dance_state_t *state, void *user_data);
void X_LFT_ARROW_reset(qk_tap_dance_state_t *state, void *user_data);

void X_RT_ARROW_finished(qk_tap_dance_state_t *state, void *user_data);
void X_RT_ARROW_reset(qk_tap_dance_state_t *state, void *user_data);

void X_KC_HOME_finished(qk_tap_dance_state_t *state, void *user_data);
void X_KC_HOME_reset(qk_tap_dance_state_t *state, void *user_data);

void X_KC_END_finished(qk_tap_dance_state_t *state, void *user_data);
void X_KC_END_reset(qk_tap_dance_state_t *state, void *user_data);

//============= end spatdsg - tap dance press and hold














enum custom_keycodes {
  RGB_SLD = EZ_SAFE_RANGE,
  HSV_206_255_255,
  HSV_86_255_128,
  HSV_27_255_255,
};

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
  [0] = LAYOUT_ergodox_pretty(
    KC_TAB,         KC_1,           KC_2,           KC_3,           KC_4,           KC_5,           TG(1),                                          KC_PGDOWN,      KC_6,           KC_7,           KC_8,           KC_9,           KC_0,           KC_MINUS,
    KC_DELETE,      KC_Q,           KC_W,           KC_F,           KC_P,           KC_B,           KC_LBRACKET,                                    KC_RBRACKET,    KC_J,           KC_L,           KC_U,           KC_Y,           TD(SEMICOLON_PRESSANDHOLD_COLON),      KC_BSLASH,
    KC_BSPACE,      KC_A,           KC_R,           KC_S,           KC_T,           KC_G,                                                                           KC_K,           KC_N,           KC_E,           KC_O,           KC_I,           KC_QUOTE,
    KC_LSHIFT,      KC_Z,           KC_X,           KC_C,           KC_D,           KC_V,           KC_LALT,                                        TG(1),          KC_M,           KC_H,           KC_COMMA,       KC_DOT,         KC_SLASH,       KC_RSHIFT,
    KC_GRAVE,       WEBUSB_PAIR,    KC_TRANSPARENT, KC_LEFT,        KC_RIGHT,                                                                                                       KC_UP,          KC_DOWN,        KC_LBRACKET,    KC_RBRACKET,    MO(1),
                                                                                                    KC_LGUI,        TD(X_KC_HOME),        KC_LALT,        KC_EQUAL,
                                                                                                                    TD(X_KC_END),         KC_PGUP,
                                                                                    LCTL_T(KC_ESCAPE),KC_TAB,       TD(X_KC_END),         KC_PGDOWN,      KC_ENTER,       KC_SPACE
  ),
  [1] = LAYOUT_ergodox_pretty(
    KC_TRANSPARENT, KC_F1,          KC_F2,          KC_F3,          KC_F4,          KC_F5,          KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_F6,          KC_F7,          KC_F8,          KC_F9,          KC_F10,         KC_F11,
    KC_TRANSPARENT, KC_EXLM,        KC_AT,          KC_LCBR,        KC_RCBR,        KC_PIPE,        KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_UP,          KC_7,           KC_UP,          KC_9,           KC_ASTR,        KC_F12,
    KC_TAB,         KC_HASH,        KC_DLR,         KC_LPRN,        KC_RPRN,        KC_GRAVE,                                                                       KC_DOWN,        KC_LEFT,        KC_DOWN,        KC_RIGHT,       KC_PLUS,        KC_TRANSPARENT,
    KC_TRANSPARENT, KC_PERC,        KC_CIRC,        KC_LBRACKET,    KC_RBRACKET,    KC_TILD,        KC_LSHIFT,                                      KC_TRANSPARENT, KC_AMPR,        KC_1,           KC_2,           KC_3,           KC_BSLASH,      KC_TRANSPARENT,
    KC_TRANSPARENT, KC_EQUAL,       KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                                                                                 KC_TRANSPARENT, KC_DOT,         KC_0,           KC_EQUAL,       KC_TRANSPARENT,
                                                                                                    RGB_MOD,        HSV_206_255_255,RGB_TOG,        RGB_SLD,
                                                                                                                    HSV_86_255_128, KC_TRANSPARENT,
                                                                                    KC_LCTRL,       RGB_VAI,        HSV_27_255_255, KC_TRANSPARENT, RGB_HUD,        RGB_HUI
  ),
  [2] = LAYOUT_ergodox_pretty(
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, RESET,
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_MS_UP,       KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,
    KC_TRANSPARENT, KC_TRANSPARENT, KC_MS_LEFT,     KC_MS_DOWN,     KC_MS_RIGHT,    KC_TRANSPARENT,                                                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_MEDIA_PLAY_PAUSE,
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_MEDIA_PREV_TRACK,KC_MEDIA_NEXT_TRACK,KC_TRANSPARENT, KC_TRANSPARENT,
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_MS_BTN1,     KC_MS_BTN2,                                                                                                     KC_AUDIO_VOL_UP,KC_AUDIO_VOL_DOWN,KC_AUDIO_MUTE,  KC_TRANSPARENT, KC_TRANSPARENT,
                                                                                                    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,
                                                                                                                    KC_TRANSPARENT, KC_TRANSPARENT,
                                                                                    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_WWW_BACK
  ),
};


extern bool g_suspend_state;
extern rgb_config_t rgb_matrix_config;

void keyboard_post_init_user(void) {
  rgb_matrix_enable();
}

const uint8_t PROGMEM ledmap[][DRIVER_LED_TOTAL][3] = {
    [0] = { {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0} },

    [1] = { {0,240,213}, {0,240,213}, {0,240,213}, {0,240,213}, {0,240,213}, {0,240,213}, {0,240,213}, {0,240,213}, {0,240,213}, {0,240,213}, {0,240,213}, {0,240,213}, {0,240,213}, {0,240,213}, {0,240,213}, {0,240,213}, {0,240,213}, {0,240,213}, {0,240,213}, {0,240,213}, {0,240,213}, {0,240,213}, {0,240,213}, {0,240,213}, {0,240,213}, {0,240,213}, {0,240,213}, {0,240,213}, {0,240,213}, {0,240,213}, {0,240,213}, {0,240,213}, {0,240,213}, {0,240,213}, {0,240,213}, {0,240,213}, {0,240,213}, {0,240,213}, {0,240,213}, {0,240,213}, {0,240,213}, {0,240,213}, {0,240,213}, {0,240,213}, {0,240,213}, {0,240,213}, {0,240,213}, {0,240,213} },

    [2] = { {79,244,176}, {79,244,176}, {79,244,176}, {79,244,176}, {79,244,176}, {79,244,176}, {79,244,176}, {79,244,176}, {79,244,176}, {79,244,176}, {79,244,176}, {79,244,176}, {79,244,176}, {79,244,176}, {79,244,176}, {79,244,176}, {79,244,176}, {79,244,176}, {79,244,176}, {79,244,176}, {79,244,176}, {79,244,176}, {79,244,176}, {79,244,176}, {79,244,176}, {79,244,176}, {79,244,176}, {79,244,176}, {79,244,176}, {79,244,176}, {79,244,176}, {79,244,176}, {79,244,176}, {79,244,176}, {79,244,176}, {79,244,176}, {79,244,176}, {79,244,176}, {79,244,176}, {79,244,176}, {79,244,176}, {79,244,176}, {79,244,176}, {79,244,176}, {79,244,176}, {79,244,176}, {79,244,176}, {79,244,176} },

};

void set_layer_color(int layer) {
  for (int i = 0; i < DRIVER_LED_TOTAL; i++) {
    HSV hsv = {
      .h = pgm_read_byte(&ledmap[layer][i][0]),
      .s = pgm_read_byte(&ledmap[layer][i][1]),
      .v = pgm_read_byte(&ledmap[layer][i][2]),
    };
    if (!hsv.h && !hsv.s && !hsv.v) {
        rgb_matrix_set_color( i, 0, 0, 0 );
    } else {
        RGB rgb = hsv_to_rgb( hsv );
        float f = (float)rgb_matrix_config.hsv.v / UINT8_MAX;
        rgb_matrix_set_color( i, f * rgb.r, f * rgb.g, f * rgb.b );
    }
  }
}

void rgb_matrix_indicators_user(void) {
  if (g_suspend_state || keyboard_config.disable_layer_led) { return; }
  switch (biton32(layer_state)) {
    case 0:
      set_layer_color(0);
      break;
    case 1:
      set_layer_color(1);
      break;
    case 2:
      set_layer_color(2);
      break;
   default:
    if (rgb_matrix_get_flags() == LED_FLAG_NONE)
      rgb_matrix_set_color_all(0, 0, 0);
    break;
  }
}

bool process_record_user(uint16_t keycode, keyrecord_t *record) {
  switch (keycode) {
    case RGB_SLD:
      if (record->event.pressed) {
        rgblight_mode(1);
      }
      return false;
    case HSV_206_255_255:
      if (record->event.pressed) {
        rgblight_mode(1);
        rgblight_sethsv(206,255,255);
      }
      return false;
    case HSV_86_255_128:
      if (record->event.pressed) {
        rgblight_mode(1);
        rgblight_sethsv(86,255,128);
      }
      return false;
    case HSV_27_255_255:
      if (record->event.pressed) {
        rgblight_mode(1);
        rgblight_sethsv(27,255,255);
      }
      return false;
  }
  return true;
}

uint32_t layer_state_set_user(uint32_t state) {

  uint8_t layer = biton32(state);

  ergodox_board_led_off();
  ergodox_right_led_1_off();
  ergodox_right_led_2_off();
  ergodox_right_led_3_off();
  switch (layer) {
    case 1:
      ergodox_right_led_1_on();
      break;
    case 2:
      ergodox_right_led_2_on();
      break;
    case 3:
      ergodox_right_led_3_on();
      break;
    case 4:
      ergodox_right_led_1_on();
      ergodox_right_led_2_on();
      break;
    case 5:
      ergodox_right_led_1_on();
      ergodox_right_led_3_on();
      break;
    case 6:
      ergodox_right_led_2_on();
      ergodox_right_led_3_on();
      break;
    case 7:
      ergodox_right_led_1_on();
      ergodox_right_led_2_on();
      ergodox_right_led_3_on();
      break;
    default:
      break;
  }
  return state;
};


//=============  spatdsg - tap dance press and hold

//====================
/* Return an integer that corresponds to what kind of tap dance should be executed.
 *
 * How to figure out tap dance state: interrupted and pressed.
 *
 * Interrupted: If the state of a dance dance is "interrupted", that means that another key has been hit
 *  under the tapping term. This is typically indicitive that you are trying to "tap" the key.
 *
 * Pressed: Whether or not the key is still being pressed. If this value is true, that means the tapping term
 *  has ended, but the key is still being pressed down. This generally means the key is being "held".
 *
 * One thing that is currenlty not possible with qmk software in regards to tap dance is to mimic the "permissive hold"
 *  feature. In general, advanced tap dances do not work well if they are used with commonly typed letters.
 *  For example "A". Tap dances are best used on non-letter keys that are not hit while typing letters.
 *
 * Good places to put an advanced tap dance:
 *  z,q,x,j,k,v,b, any function key, home/end, comma, semi-colon
 *
 * Criteria for "good placement" of a tap dance key:
 *  Not a key that is hit frequently in a sentence
 *  Not a key that is used frequently to double tap, for example 'tab' is often double tapped in a terminal, or
 *    in a web form. So 'tab' would be a poor choice for a tap dance.
 *  Letters used in common words as a double. For example 'p' in 'pepper'. If a tap dance function existed on the
 *    letter 'p', the word 'pepper' would be quite frustating to type.
 *
 * For the third point, there does exist the 'DOUBLE_SINGLE_TAP', however this is not fully tested
 *
 */
int cur_dance (qk_tap_dance_state_t *state) {
  if (state->count == 1) {
    if (state->interrupted || !state->pressed)  return SINGLE_TAP;
    //key has not been interrupted, but they key is still held. Means you want to send a 'HOLD'.
    else return SINGLE_HOLD;
  }
  else if (state->count == 2) {
    /*
     * DOUBLE_SINGLE_TAP is to distinguish between typing "pepper", and actually wanting a double tap
     * action when hitting 'pp'. Suggested use case for this return value is when you want to send two
     * keystrokes of the key, and not the 'double tap' action/macro.
    */
    if (state->interrupted) return DOUBLE_SINGLE_TAP;
    else if (state->pressed) return DOUBLE_HOLD;
    else return DOUBLE_TAP;
  }
  //Assumes no one is trying to type the same letter three times (at least not quickly).
  //If your tap dance key is 'KC_W', and you want to type "www." quickly - then you will need to add
  //an exception here to return a 'TRIPLE_SINGLE_TAP', and define that enum just like 'DOUBLE_SINGLE_TAP'
  if (state->count == 3) {
    if (state->interrupted || !state->pressed)  return TRIPLE_TAP;
    else return TRIPLE_HOLD;
  }
  else return 8; //magic number. At some point this method will expand to work for more presses
}

//instanalize an instance of 'tap' for the 'x' tap dance.
static tap xtap_state = {
  .is_press_action = true,
  .state = 0
};


//SPATDSG ===================== actual definitions of what to do here
//https://www.reddit.com/r/olkb/comments/73ue11/qmk_need_help_with_compiling_the_hex/
//You need to expand out register_code(KC_RIGHT_ANGLE_BRACKET); into:
//register_code(KC_LSFT);
//register_code(KC_DOT);

void SEMICOLON_PRESSANDHOLD_COLON_finished (qk_tap_dance_state_t *state, void *user_data) {
  xtap_state.state = cur_dance(state);
  switch (xtap_state.state) {
    case SINGLE_TAP: register_code(KC_SCOLON); break;  // on single tap it will print a semicolon
    case SINGLE_HOLD: register_code(KC_LSFT); register_code(KC_SCOLON);
    //case DOUBLE_TAP: register_code(KC_ESC); break;
    //case DOUBLE_HOLD: register_code(KC_LALT); break;
    //case DOUBLE_SINGLE_TAP: register_code(KC_X); unregister_code(KC_X); register_code(KC_X);
    //Last case is for fast typing. Assuming your key is `f`:
    //For example, when typing the word `buffer`, and you want to make sure that you send `ff` and not `Esc`.
    //In order to type `ff` when typing fast, the next character will have to be hit within the `TAPPING_TERM`, which by default is 200ms.
  }
}


void SEMICOLON_PRESSANDHOLD_COLON_reset (qk_tap_dance_state_t *state, void *user_data) {
  switch (xtap_state.state) {
    case SINGLE_TAP: unregister_code(KC_SCOLON); break;
    case SINGLE_HOLD: unregister_code(KC_LSFT); unregister_code(KC_SCOLON);
    //case DOUBLE_TAP: unregister_code(KC_ESC); break;
    //case DOUBLE_HOLD: unregister_code(KC_LALT);
    //case DOUBLE_SINGLE_TAP: unregister_code(KC_X);
  }
  xtap_state.state = 0;
}


// END SPATDSG ===================== actual definitions of what to do here



//============================= arrows
//press and hold - does normal repeat pattern
//double tap - ctrl+arrow will jump blocks of text
//double tap hold - ctrl+shift+arrow will select blocks of text

void X_LFT_ARROW_finished (qk_tap_dance_state_t *state, void *user_data) {
  xtap_state.state = cur_dance(state);
  switch (xtap_state.state) {
    case SINGLE_TAP: register_code(KC_LEFT); break;  // on single tap it will print a semicolon
    //case SINGLE_HOLD: register_code(KC_LEFT);
    case DOUBLE_TAP: register_code(KC_LCTRL);register_code(KC_LEFT); break;
    case DOUBLE_HOLD: register_code(KC_LCTRL);register_code(KC_LSHIFT);register_code(KC_LEFT); break;
    case DOUBLE_SINGLE_TAP: register_code(KC_LEFT); unregister_code(KC_LEFT); register_code(KC_LEFT);
    //Last case is for fast typing. Assuming your key is `f`:
    //For example, when typing the word `buffer`, and you want to make sure that you send `ff` and not `Esc`.
    //In order to type `ff` when typing fast, the next character will have to be hit within the `TAPPING_TERM`, which by default is 200ms.
  }
}


void X_LFT_ARROW_reset (qk_tap_dance_state_t *state, void *user_data) {
  switch (xtap_state.state) {
    case SINGLE_TAP: unregister_code(KC_LEFT); break;  // on single tap it will print a semicolon
    //case SINGLE_HOLD: register_code(KC_LEFT);
    case DOUBLE_TAP: unregister_code(KC_LCTRL);unregister_code(KC_LEFT); break;
    case DOUBLE_HOLD: unregister_code(KC_LCTRL);unregister_code(KC_LSHIFT);unregister_code(KC_LEFT); break;
    case DOUBLE_SINGLE_TAP: unregister_code(KC_LEFT);
  
  }
  xtap_state.state = 0;
}


void X_RT_ARROW_finished (qk_tap_dance_state_t *state, void *user_data) {
  xtap_state.state = cur_dance(state);
  switch (xtap_state.state) {
    case SINGLE_TAP: register_code(KC_RIGHT); break;  // on single tap it will print a semicolon
    //case SINGLE_HOLD: register_code(KC_LEFT);
    case DOUBLE_TAP: register_code(KC_LCTRL);register_code(KC_RIGHT); break;
    case DOUBLE_HOLD: register_code(KC_LCTRL);register_code(KC_LSHIFT);register_code(KC_RIGHT); break;
    case DOUBLE_SINGLE_TAP: register_code(KC_RIGHT); unregister_code(KC_RIGHT); register_code(KC_RIGHT);
    //Last case is for fast typing. Assuming your key is `f`:
    //For example, when typing the word `buffer`, and you want to make sure that you send `ff` and not `Esc`.
    //In order to type `ff` when typing fast, the next character will have to be hit within the `TAPPING_TERM`, which by default is 200ms.
  }
}


void X_RT_ARROW_reset (qk_tap_dance_state_t *state, void *user_data) {
  switch (xtap_state.state) {
    case SINGLE_TAP: unregister_code(KC_RIGHT); break;  // on single tap it will print a semicolon
    //case SINGLE_HOLD: register_code(KC_LEFT);
    case DOUBLE_TAP: unregister_code(KC_LCTRL);unregister_code(KC_RIGHT); break;
    case DOUBLE_HOLD: unregister_code(KC_LCTRL);unregister_code(KC_LSHIFT);unregister_code(KC_RIGHT); break;
    case DOUBLE_SINGLE_TAP: unregister_code(KC_RIGHT);
  
  }
  xtap_state.state = 0;
}


//============================= end arrows

















//============================= HOME AND END
//press and hold - will *select all* to home or all to end
// double tap will jump to home or end of document


//home key
void X_KC_HOME_finished (qk_tap_dance_state_t *state, void *user_data) {
  xtap_state.state = cur_dance(state);
  switch (xtap_state.state) {
    case SINGLE_TAP: register_code(KC_HOME); break; 
    case SINGLE_HOLD: register_code(KC_LCTRL);register_code(KC_LSHIFT);register_code(KC_HOME); break;//press and hold - will *select all* to home or all to end
    case DOUBLE_TAP: register_code(KC_LCTRL);register_code(KC_HOME); break; //double tap will jump to home or end of document
    //case DOUBLE_HOLD: register_code(KC_LCTRL);register_code(KC_LSHIFT);register_code(KC_LEFT); break;
    case DOUBLE_SINGLE_TAP: register_code(KC_HOME); unregister_code(KC_HOME); register_code(KC_HOME);
    //Last case is for fast typing. Assuming your key is `f`:
    //For example, when typing the word `buffer`, and you want to make sure that you send `ff` and not `Esc`.
    //In order to type `ff` when typing fast, the next character will have to be hit within the `TAPPING_TERM`, which by default is 200ms.
  }
}


void X_KC_HOME_reset (qk_tap_dance_state_t *state, void *user_data) {
  switch (xtap_state.state) {
    case SINGLE_TAP: unregister_code(KC_HOME); break; 
    case SINGLE_HOLD: unregister_code(KC_LCTRL);unregister_code(KC_LSHIFT);unregister_code(KC_HOME); break;
    case DOUBLE_TAP: unregister_code(KC_LCTRL);unregister_code(KC_HOME); break;
    //case DOUBLE_HOLD: register_code(KC_LCTRL);register_code(KC_LSHIFT);register_code(KC_LEFT); break;
    case DOUBLE_SINGLE_TAP: unregister_code(KC_HOME);
  
  }
  xtap_state.state = 0;
}

//end key
void X_KC_END_finished (qk_tap_dance_state_t *state, void *user_data) {
  xtap_state.state = cur_dance(state);
  switch (xtap_state.state) {
    case SINGLE_TAP: register_code(KC_END); break; 
    case SINGLE_HOLD: register_code(KC_LCTRL);register_code(KC_LSHIFT);register_code(KC_END); break;//press and hold - will *select all* to home or all to end
    case DOUBLE_TAP: register_code(KC_LCTRL);register_code(KC_END); break; //double tap will jump to home or end of document
    //case DOUBLE_HOLD: register_code(KC_LCTRL);register_code(KC_LSHIFT);register_code(KC_LEFT); break;
    case DOUBLE_SINGLE_TAP: register_code(KC_END); unregister_code(KC_END); register_code(KC_END);
    //Last case is for fast typing. Assuming your key is `f`:
    //For example, when typing the word `buffer`, and you want to make sure that you send `ff` and not `Esc`.
    //In order to type `ff` when typing fast, the next character will have to be hit within the `TAPPING_TERM`, which by default is 200ms.
  }
}


void X_KC_END_reset (qk_tap_dance_state_t *state, void *user_data) {
  switch (xtap_state.state) {
    case SINGLE_TAP: unregister_code(KC_END); break; 
    case SINGLE_HOLD: unregister_code(KC_LCTRL);unregister_code(KC_LSHIFT);unregister_code(KC_END); break;
    case DOUBLE_TAP: unregister_code(KC_LCTRL);unregister_code(KC_END); break;
    //case DOUBLE_HOLD: register_code(KC_LCTRL);register_code(KC_LSHIFT);register_code(KC_LEFT); break;
    case DOUBLE_SINGLE_TAP: unregister_code(KC_END);
  
  }
  xtap_state.state = 0;
}


//============================= end HOME AND END

/*

---- THIS WAS FROM THE ORIGINAL EXAMPLE ---

void x_finished (qk_tap_dance_state_t *state, void *user_data) {
  xtap_state.state = cur_dance(state);
  switch (xtap_state.state) {
    case SINGLE_TAP: register_code(KC_X); break;
    case SINGLE_HOLD: register_code(KC_LCTRL); break;
    case DOUBLE_TAP: register_code(KC_ESC); break;
    case DOUBLE_HOLD: register_code(KC_LALT); break;
    case DOUBLE_SINGLE_TAP: register_code(KC_X); unregister_code(KC_X); register_code(KC_X);
    //Last case is for fast typing. Assuming your key is `f`:
    //For example, when typing the word `buffer`, and you want to make sure that you send `ff` and not `Esc`.
    //In order to type `ff` when typing fast, the next character will have to be hit within the `TAPPING_TERM`, which by default is 200ms.
  }
}

void x_reset (qk_tap_dance_state_t *state, void *user_data) {
  switch (xtap_state.state) {
    case SINGLE_TAP: unregister_code(KC_X); break;
    case SINGLE_HOLD: unregister_code(KC_LCTRL); break;
    case DOUBLE_TAP: unregister_code(KC_ESC); break;
    case DOUBLE_HOLD: unregister_code(KC_LALT);
    case DOUBLE_SINGLE_TAP: unregister_code(KC_X);
  }
  xtap_state.state = 0;
}

*/




/*
enum {
  X_CTL = 0,
  SEMICOLON_PRESSANDHOLD_COLON = 1,
  X_LFT_ARROW = 2,
  X_RT_ARROW = 3,
  X_KC_HOME = 4,
  X_KC_END = 5

};

*/

//see https://jayliu50.github.io/qmk-cheatsheet/
qk_tap_dance_action_t tap_dance_actions[] = {
  //[X_CTL]     = ACTION_TAP_DANCE_FN_ADVANCED(NULL,x_finished, x_reset),
  [SEMICOLON_PRESSANDHOLD_COLON]     = ACTION_TAP_DANCE_FN_ADVANCED(NULL,SEMICOLON_PRESSANDHOLD_COLON_finished, SEMICOLON_PRESSANDHOLD_COLON_reset),
  [X_LFT_ARROW]   = ACTION_TAP_DANCE_FN_ADVANCED(NULL,X_LFT_ARROW_finished, X_LFT_ARROW_reset),
  [X_RT_ARROW]   = ACTION_TAP_DANCE_FN_ADVANCED(NULL,X_RT_ARROW_finished, X_RT_ARROW_reset),
  [X_KC_HOME]  = ACTION_TAP_DANCE_FN_ADVANCED(NULL,X_KC_HOME_finished, X_KC_HOME_reset),
  [X_KC_END]  = ACTION_TAP_DANCE_FN_ADVANCED(NULL,X_KC_END_finished, X_KC_END_reset)
};

//============= end spatdsg - tap dance press and hold
