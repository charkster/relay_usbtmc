
#ifndef USBTMC_APP_H
#define USBTMC_APP_H

void     usbtmc_app_task_iter(void);

void     gpio_setup(void);

char * get_value(char *in_string);
char * get_command(char *in_string, char *ptr_value);
void samd21_unique_id( char * id_buff );

#endif
