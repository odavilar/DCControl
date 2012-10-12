#include <stdio.h>
#include <ncurses.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "ixpio.h"

#define WIDTH 30
#define HEIGHT 10 

int startx = 0;
int starty = 0;

int main(){	
	WINDOW *menu_win;
	int highlight = 1;
	int choice = 0;
	int c;

	int fd;
	char *dev_file;
	ixpio_reg_t reg0, reg1;

	dev_file = "/dev/ixpio1";

	/* open device file */
	fd = open(dev_file, O_RDWR);
	if (fd < 0) {
		printf("Failure of open device file \"%s.\"\n", dev_file);
		return FAILURE;
	}

	reg0.id = IXPIO_PCB;
	reg0.value = 0x01;
	if (ioctl(fd, IXPIO_WRITE_REG, &reg0)) {
		printf("Failure of ioctl command IXPIO_WRITE_REG IXPIO_PC.\n");
		close(fd);
		return FAILURE;
	}

	reg1.id = IXPIO_PCC;
	reg1.value = 0x01;
	if (ioctl(fd, IXPIO_WRITE_REG, &reg1)) {
		printf("Failure of ioctl command IXPIO_WRITE_REG IXPIO_PC.\n");
		close(fd);
		return FAILURE;
	}

	initscr();
	clear();
	noecho();
	cbreak();	/* Line buffering disabled. pass on everything */
	startx = 0;/*(80 - WIDTH) / 2;*/
	starty = 1;/*(24 - HEIGHT) / 2;*/

	menu_win = newwin(HEIGHT, WIDTH, starty, startx);
	keypad(menu_win, TRUE);
	mvprintw(0, 0, "Use arrow keys to go XPOS, XNEG, ZPOS, ZNEG.");
	refresh();
	while(choice == 0)
	{	
		c = wgetch(menu_win);
		switch(c)
		{	case KEY_UP:
			mvprintw(1, 0, "XPOS");
			clrtoeol();
			reg0.id = IXPIO_P6;
			reg0.value = 2;
			if( ioctl(fd, IXPIO_WRITE_REG, &reg0) ) {
				puts("Failure of IXPIO_READ/WRITE_REG");
				close(fd);
				return FAILURE;
			}
			refresh();
			break;
			case KEY_DOWN:
			mvprintw(1, 0, "XNEG");
			clrtoeol();
			reg0.id = IXPIO_P6;
			reg0.value = 1;
			if( ioctl(fd, IXPIO_WRITE_REG, &reg0) ) {
				puts("Failure of IXPIO_READ/WRITE_REG");
				close(fd);
				return FAILURE;
			}
			refresh();
			break;
			case KEY_LEFT:
			mvprintw(1, 0, "ZPOS");
			clrtoeol();
			reg1.id = IXPIO_P3;
			reg1.value = 1;
			if( ioctl(fd, IXPIO_WRITE_REG, &reg1) ) {
				puts("Failure of IXPIO_READ/WRITE_REG");
				close(fd);
				return FAILURE;
			}
			refresh();
			break;
			case KEY_RIGHT:
			mvprintw(1, 0, "ZNEG");
			clrtoeol();
			reg1.id = IXPIO_P3;
			reg1.value = 2;
			if( ioctl(fd, IXPIO_WRITE_REG, &reg1) ) {
				puts("Failure of IXPIO_READ/WRITE_REG");
				close(fd);
				return FAILURE;
			}
			refresh();
			break;
			case 27:
			choice = 1;
			break;
			case 's':
			mvprintw(1, 0, "STOP");
			clrtoeol();
			reg0.id = IXPIO_P3;
			reg0.value = 0;
			reg1.id = IXPIO_P6;
			reg1.value = 0;
			if( ioctl(fd, IXPIO_WRITE_REG, &reg1) || ioctl(fd, IXPIO_WRITE_REG, &reg0)) {
				puts("Failure of IXPIO_READ/WRITE_REG");
				close(fd);
				return FAILURE;
			}
			refresh();
			break;
			default:
			//mvprintw(1, 0, "Charcter pressed is = %3d Hopefully it can be printed as '%c'", c, c);
			refresh();
			break;
		}
		/*if(choice != 0)	
		  break;*/
	}	
	clrtoeol();
	refresh();
	endwin();
	return 0;
}
