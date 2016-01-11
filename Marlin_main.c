/*
* Marlin ported to work on the Intel Edison
*
* Sean Smith 2015
*/

#include "Marlin.h"
#include "Configuration.h"
#include <stdio.h>
#include <stdlib.h>

#include <fcntl.h>
#include <sys/stat.h>


static char cmdbuffer[MAX_CMD_SIZE];

bool Stopped = false;

/*typedef struct line {
	char *gcode;
	char *params;
	line *next;
} line;*/

char *file_buf = NULL;
int file_size;
int current_read = 0;


int setup(char *);
void loop();
//int parse(char *line, line *l);
void get_command();

int main(int argc, char *argv[]) {

  if (argc != 2) {
    printf("Wrong number of arguments provided... Provided %d instead \
            of 2\nmarlin /path/to/file\n", argc);
    exit(1);
  }

  int file = setup(argv[1]);

  loop(file);

  return 0;
}

int setup(char *path)
{
  int file;

  struct stat s;
  if (stat(path, &s) == -1) {
    printf("Error stating %s\n", path);
    exit(1);
  }

  file_size = s.st_size;

  if ((file = open(path, O_RDONLY))) {
    printf("Error opening %s\n", path);
    exit(1);
  }

  file_buf = (char *)malloc(file_size);
  if (read(file, file_buf, file_size) < file_size) {
    printf("Error reading file\n");
    exit(1);
  }

  return file;
}

/*
char *readline(char **line, int *len, int fd) {

	int count = 0;



	while (1) {
		// Check if you need to read more chars
		if (current_read >= buffer_size) {
			int read_len = read(fd, line_buf, MAX_LINE_BUF);
			if (read_len == -1) {
				printf("read error\n");
				exit(1);
			}
			buffer_size = read_len;
			current_read = 0;
		}

		if (line_buf[current_read] == '\n') {
			break;
		}

		count++;

		current_read++;
	}

	*line = malloc(count * sizeof(char));



	return ret;
}
*/

void loop(int fd) {


	// For line in file
/*
	char *line = NULL;
	int len = 0;
	ssize_t read;


	while ((read = readline(&line, &len, fd)) != -1) {
		printf("line of size %lu = %s\n", len, line);
		line *l = calloc(sizeof(line));
		parse(line, l);
		len = 0;
	}*/

  
  while (1) {
    get_command();
  }
}

void get_command()
{
  char serial_char;
  int serial_count = 0;
  bool comment_mode = false;
  char *strchr_pointer;

  while (current_read < file_size) {
    serial_char = file_buf[current_read++];
    if(serial_char == '\n' ||
       serial_char == '\r' ||
       (serial_char == ':' && comment_mode == false) ||
       serial_count >= (MAX_CMD_SIZE - 1) )
    {
      if(!serial_count) { //if empty line
        comment_mode = false; //for new command
        return;
      }
      cmdbuffer[serial_count] = 0;  //terminate string
      if(!comment_mode){
        comment_mode = false; //for new command

        if(strchr(cmdbuffer, 'N') != NULL)
        {
          printf("line number support needed\n");
          exit(1);
        }
        if((strchr(cmdbuffer, '*') != NULL))
        {
          printf("checksum support needed\n");
          exit(1);
        }

        if((strchr(cmdbuffer, 'G') != NULL)){
          strchr_pointer = strchr(cmdbuffer, 'G');
          switch((int)((strtod(&cmdbuffer[strchr_pointer - cmdbuffer + 1], NULL)))){
          case 0:
          case 1:
          case 2:
          case 3:
            if(Stopped == false) { // If printer is stopped by an error the G[0-3] codes are ignored.
            }
            else {
              printf("MSG_ERR_STOPPED\n");
            }
            break;
          default:
            break;
          }
        }
        serial_count = 0; //clear buffer
      }
      else
      {
        if(serial_char == ';') comment_mode = true;
        if(!comment_mode) cmdbuffer[serial_count++] = serial_char;
      }
    }
  }
}

/*
int parse(char *line, line *l) {

}
*/

/* vi: set et sw=2 sts=2: */
