/*
* Marlin ported to work on the Intel Edison
*
* Sean Smith 2015
*/

#include "Marlin.h"
#include <stdio.h>
//#include <stdlib>

#include <fcntl.h>
#include <sys/stat.h>


typedef struct line {
	char *gcode;
	char *params;
	line *next;
} line;

#define MAX_LINE_BUF 20


off_t MAX_BUF;

char line_buf[MAX_LINE_BUF];
int current_read = 0;
int buffer_size = 0;


FILE *setup(char *path);
void loop(FILE *fd);
int parse(char *line, line *l);

int main(int argc, char *argv[]) {

	if (argc != 2) {
		printf("Wrong number of arguments provided... Provided %d instead of 2\nmarlin /path/to/file\n", argc);
		exit(1);
	}

	FILE *file = setup(argv[1]);

	loop(file);

	// while (1) {

	// 	loop(file);
	// }
	return 0;
}

FILE *setup(char *path)
{
	int file;

	struct stat s;
	if (stat(path, &s) == -1) {
		printf("Error stating %s\n", path);
		exit(1);
	}

	MAX_BUF = s->st_size;

	if ((file = open(path, O_RDONLY)) == NULL) {
		printf("Error opening %s\n", path);
		exit(1);
	}

	return file;

}

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

void loop(int fd) {


	// For line in file

	char *line = NULL;
	int len = 0;
	ssize_t read;


	while ((read = readline(&line, &len, fd)) != -1) {
		printf("line of size %lu = %s\n", len, line);
		line *l = calloc(sizeof(line));
		parse(line, l);
		len = 0;
	}
}


int parse(char *line, line *l) {

	


}



