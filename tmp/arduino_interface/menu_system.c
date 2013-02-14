#include "Arduino.h"
#include <Wire.h>

/*
#include <stdio.h>
#include <stdlib.h>
*/

/*
#include <termios.h>
#include <unistd.h>

#include <string.h>
*/

#include "menu_system.h"

cmd_t cmd = {0};

void cmd_history_commit()
{
  int i, j, k, pos;

  // get destination position into static history to copy
  // current command into
  pos = ( cmd.history_start + cmd.history_n ) % CMD_HISTORY_LEN;
  if ( cmd.history_n < CMD_HISTORY_LEN )
    cmd.history_n ++;

  if (cmd.history_n == CMD_HISTORY_LEN)
    cmd.history_start = (cmd.history_start + 1) % CMD_HISTORY_LEN;

  // copy current command into static history
  k = cmd.history_scratch_pos;
  for (i=0; i < CMD_BUF_LEN; i++)
    cmd.history[ pos ][i] = cmd.history_scratch[ k ][i];

  // copy static history into scratch
  for (i=0; i < CMD_HISTORY_LEN; i++)
    for (j=0; j < CMD_BUF_LEN; j++)
      cmd.history_scratch[i][j] = cmd.history[i][j];

  // update scratch position
  cmd.history_scratch_pos = (cmd.history_start + cmd.history_n) % CMD_HISTORY_LEN;

  // keep buf as our pointer to our active command line to simplify things
  cmd.buf = cmd.history_scratch[ cmd.history_scratch_pos ];
  cmd.buf[0] = '\0';
  cmd.buf_pos = 0;
  cmd.buf_n = 0;


  Serial.write("$ ");
  //write( STDOUT_FILENO, "$ ", 2);

}

void cmd_verbose_print(void)
{
  int i;
  int s, e, p;
  printf("cmd history_start %i, history_n %i\n\r",
    cmd.history_start, cmd.history_n);

  printf("history:\r\n");
  s = cmd.history_start;
  e = (cmd.history_start + cmd.history_n + CMD_HISTORY_LEN - 1) % CMD_HISTORY_LEN;
  for (i=0; i<cmd.history_n; i++)
  {
    p = (cmd.history_start + i) % CMD_HISTORY_LEN;
    printf("  [%i] '%s'\r\n", p, cmd.history[p]);
  }
  //for (i = s; i != e; i = (i + 1) % CMD_HISTORY_LEN ) printf("  [%i] '%s'\r\n", i, cmd.history[i]);

  printf("\r\nhistory_scratch (%i):\r\n", cmd.history_scratch_pos);
  for (i=0; i<cmd.history_n; i++)
  {
    p = (cmd.history_start + i) % CMD_HISTORY_LEN;
    printf(" %c[%i] '%s'\r\n", (p == cmd.history_scratch_pos) ? '*' : ' ', p, cmd.history_scratch[p]);
  }
  //for (i = s; i != e; i = (i + 1) % CMD_HISTORY_LEN ) printf(" %c[%i] '%s'\r\n", (i == cmd.history_scratch_pos) ? '*' : ' ', i, cmd.history[i]);


  printf("buf_n %i, buf_pos %i '%s'\r\n", cmd.buf_n, cmd.buf_pos, cmd.buf);
  printf("in_esc %i, esc_buf(%i,%i,%i) esc_buf_n %i\n\r",
      cmd.in_esc, cmd.esc_buf[0], cmd.esc_buf[1], cmd.esc_buf[2], cmd.esc_buf_n);
  printf("\r\n");

  fflush(stdout);
}

//--------------

void cmd_bs()
{
  int i;

  if (cmd.buf_pos == 0)
    return;

  Serial.write("\b");
  //write( STDOUT_FILENO, "\b", 1);


  for ( i = cmd.buf_pos; i < cmd.buf_n; i++)
  {
    cmd.buf[i-1] = cmd.buf[i];

    Serial.write("\b"); //????
    //write( STDOUT_FILENO, cmd.buf + i, 1);
  }

  cmd.buf_pos--;
  cmd.buf[--cmd.buf_n] = '\0';

  Serial.write(" ");
  //write( STDOUT_FILENO, " ", 1);

  for ( i = cmd.buf_n; i > cmd.buf_pos; i-- )
    Serial.write("\b");
    //write(STDOUT_FILENO, "\b", 1);

  Serial.write("\b");
  //write(STDOUT_FILENO, "\b", 1);

}

//--------------

// not implemented
void cmd_too_long(uint8_t byt)
{
}

//--------------

void cmd_add_byte(uint8_t byt)
{
  int i;

  // just in case
  if (cmd.buf_n >= (CMD_BUF_LEN-1)) 
    return;

  Serial.write(&byt, 1);  //???
  //write( STDOUT_FILENO,  &byt, 1 );

  // shift over latter characters
  for (i = cmd.buf_pos; i < cmd.buf_n; i++)
    Serial.write(cmd.buf + i , 1); //????
    //write( STDOUT_FILENO, cmd.buf + i, 1);

  // move cursor back
  for (i = cmd.buf_pos; i < cmd.buf_n; i++)
    Serial.write("\b"); 
    //write(STDOUT_FILENO, "\b", 1);

  // copy over characters in our buffer
  for (i = cmd.buf_n; i > cmd.buf_pos; i--)
    cmd.buf[i] = cmd.buf[i-1];
  cmd.buf[cmd.buf_pos++] = byt;
  cmd.buf[++cmd.buf_n] = '\0';

  if (cmd.buf_pos == (CMD_BUF_LEN-1))
  {
    Serial.write("\b"); //????
    //write(STDOUT_FILENO, "\b", 1);
    cmd.buf_pos--;
  }


}

//--------------

void cmd_cursor_left(void)
{
  //write ( STDOUT_FILENO, "cursor left\n\r", strlen("cursor left\n\r"));

  if (cmd.buf_pos == 0) return;
  cmd.buf_pos--;

  Serial.write("\b");
  //write ( STDOUT_FILENO, "\b", 1);
}

//--------------

void cmd_cursor_right(void)
{
  if (cmd.buf_pos == cmd.buf_n ) return;
  cmd.buf_pos++;

  Serial.write(cmd.buf + cmd.buf_pos - 1, 1); //????
  //write ( STDOUT_FILENO, cmd.buf + cmd.buf_pos - 1, 1);
}

//--------------

void cmd_cursor_up(void)
{

  int i, pos;

  // check if we've gotten to the first position
  pos = (cmd.history_scratch_pos + 8 - 1) % CMD_HISTORY_LEN;
  if ((cmd.history_n == CMD_HISTORY_LEN) && (pos == cmd.history_start))
    return;
  if (pos > cmd.history_n)
    return;

  // erase our current command from the terminal
  for (i=cmd.buf_pos; i<cmd.buf_n; i++)
    Serial.write(" ");
    //write(STDOUT_FILENO, " ", 1);
  for (i=0; i<cmd.buf_n; i++)
    Serial.write("\b \b");
    //write( STDOUT_FILENO, "\b \b", 3);

  // update our buffer
  cmd.buf = cmd.history_scratch[ pos ];
  cmd.buf_n = strlen((char *)cmd.history_scratch[ pos ]);
  cmd.history_scratch_pos = pos;

  cmd.buf_pos = cmd.buf_n;

  // print out our new command
  if (cmd.buf_n > 0)
    Serial.write(cmd.buf); //????
    //write( STDOUT_FILENO, cmd.buf, cmd.buf_n);

  // repositino our cursor (currently not implemented properly)
  for (i=cmd.buf_pos ; i < cmd.buf_n; i++)
    Serial.write("\b");
    //write( STDOUT_FILENO, "\b", 1);

}

//--------------

void cmd_cursor_down()
{
  int i, pos, b;

  // make sure we haven't fallen off of the edge
  b = (cmd.history_start + cmd.history_n + 1) % CMD_HISTORY_LEN;
  pos = (cmd.history_scratch_pos + 1) % CMD_HISTORY_LEN;
  if (pos == b)
    return;

  // clear out command fromt he terminal
  for (i=cmd.buf_pos; i<cmd.buf_n; i++)
    Serial.write(" ");
    //write(STDOUT_FILENO, " ", 1);
  for (i=0; i<cmd.buf_n; i++)
    Serial.write("\b \b");
    //write( STDOUT_FILENO, "\b \b", 3);

  // update our buffer
  cmd.buf = cmd.history_scratch[ pos ];
  cmd.buf_n = strlen((char *)cmd.history_scratch[ pos ]);
  cmd.history_scratch_pos = pos;

  cmd.buf_pos = cmd.buf_n;

  if (cmd.buf_n > 0)
    Serial.write(cmd.buf);
    //write( STDOUT_FILENO, cmd.buf, cmd.buf_n);

  for (i=cmd.buf_pos ; i < cmd.buf_n; i++)
    Serial.write("\b");
    //write( STDOUT_FILENO, "\b", 1);

}

//--------------

void cmd_init()
{
  cmd.history_n=0;
  cmd.buf = cmd.history_scratch[0];
}

void cmd_show_help(void)
{
  printf("help\n\r");
  printf("p    program\n\r");
  printf("e    read eeprom\n\r");
  printf("w    write eeprom\n\r");
  printf("d    get date\n\r");
  printf("s    set date\n\r");
  printf("R    reset\n\r");
  printf("h/?  help\n\r");
  printf("~    DEBUG show current command history\n\r");
  printf("v    DEBUG show command history\n\r");
  printf("Q    quit\n\r");

  fflush(stdout);

}

void cmd_exec(uint8_t *b)
{

  //Serial.write("\r\n");
  printf("\r\n");

  if (b[0] == 'p')
  {
    printf("program avr not implemented\n\r");
  }
  else if (b[0] == 'e')
  {
    //read_eeprom();
    printf("read eeprom not implemented\n\r");
  }
  else if (b[0] == 'w')
  {
    printf("write eeprom not implemented\n\r");
  }
  else if (b[0] == 'd')
  {
    printf("get date not implemented\n\r");
  }
  else if (b[0] == 's')
  {
    printf("set date not implemented\n\r");
  }
  else if (b[0] == 'R')
  {
    printf("reset not implemented\n\r");
  }
  else if (b[0] == 'v')
  {

    cmd_verbose_print();

  }
  else if ((b[0] == '\0') || (b[0] == '\r') || (b[0] == '\n'))
  {
    //printf("\n\r");
  }
  else if (b[0] == 'Q')
  {
    exit(0);
  }
  else
  {
    if ((b[0] != 'h') && (b[0] != '?'))
      printf("invalid command\n\r");
    cmd_show_help();
    //printf("help\n\rp  program\n\re  read eeprom\n\rw  write eeprom\n\rd  get date\n\rs  set date\n\rR  reset\n\r");
  }

  fflush(stdout);

}


//-----------------------------------------------


void cmd_handle_esc(uint8_t byt)
{

  cmd.esc_buf[ cmd.esc_buf_n++ ] = byt;
  if (cmd.esc_buf_n == 3)
  {

    if (cmd.esc_buf[1] == '[') 
    {
      if (cmd.esc_buf[2] == 'A')
      {
        //printf("up arrow\r\n"); fflush(stdout);

        cmd_cursor_up();


      }
      else if (cmd.esc_buf[2] == 'B')
      {
        //printf("down arrow\r\n"); fflush(stdout);

        cmd_cursor_down();
      }
      else if (cmd.esc_buf[2] == 'C')
      {
        cmd_cursor_right();
      }
      else if (cmd.esc_buf[2] == 'D')
      {
        cmd_cursor_left();
      }
    }

    cmd.in_esc = 0;
    cmd.esc_buf_n=0;

  }

}

//-----------------------------------------------

void cmd_process(char byt)
{
  char tbuf[128];

  //byt = Serial.read();

  //printf("[%i]\n\r\n\r", byt); fflush(stdout);

  sprintf(tbuf, "\r\n[%x] %i\r\n", (int)byt, (int)byt);
  //Serial.write(tbuf);
  //printf("%s", tbuf); fflush(stdout);

  // our special special character
  if (byt == '~')
  {
    printf("\r\n"); fflush(stdout);
    cmd_verbose_print();
    return;
  }

  if (cmd.in_esc)
  {
    cmd_handle_esc(byt);
    return;
  }
  else if (byt == 27) // ESC
  {
    cmd.in_esc = 1;
    cmd.esc_buf[ cmd.esc_buf_n++ ] = byt;
    return;
  }

  if (byt == BS )
  {
    cmd_bs();
    return;
  }
  else if (byt == '\r') 
  {
    cmd_exec(cmd.buf);
    cmd_history_commit();
    return;
  }
  else if (cmd.buf_n == (CMD_BUF_LEN-1))
  {
    cmd_too_long(byt);
    return;
  }

  cmd_add_byte(byt);

}


