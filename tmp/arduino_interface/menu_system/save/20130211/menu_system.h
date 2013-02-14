#ifndef MENU_SYSTEM_H
#define MENU_SYSTEM_H


#define CMD_BUF_LEN 8
#define CMD_HISTORY_LEN 8
#define BS 127

typedef unsigned char uint8_t;

typedef struct command_type {

  uint8_t history[ CMD_HISTORY_LEN ][ CMD_BUF_LEN ];
  uint8_t history_start;
  uint8_t history_n;

  uint8_t history_scratch[ CMD_HISTORY_LEN ][ CMD_BUF_LEN ];
  uint8_t history_scratch_pos;

  uint8_t *buf;
  uint8_t buf_n, buf_pos;

  uint8_t in_esc;
  uint8_t esc_buf[3];
  uint8_t esc_buf_n;
} cmd_t;

extern cmd_t cmd;

void cmd_history_commit(void);
void cmd_verbose_print(void);
void cmd_bs(void);
void cmd_too_long(uint8_t byt);
void cmd_add_byte(uint8_t byt);

void cmd_cursor_left(void);
void cmd_cursor_right(void);
void cmd_cursor_up(void);
void cmd_cursor_down(void);

void cmd_init(void);
void cmd_exec(uint8_t *b);

void cmd_handle_esc(uint8_t byt);
void cmd_process(char byt);


#endif
