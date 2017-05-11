// A device driver to control a robot gripper arm to
// reproduce the hand motion of the remote operator
//
// Written by Ryan Oechsler and David Rogers
//
// To compile: gcc -pthread RobotApp.c -o RobotApp

// ******** Include the necessary Linux headers ********

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <termios.h>

// ******** Global variable declarations ********

int fp, mode;

// ******** Function prototypes ********

void setGripperAngle(int);
int  getHandAngle(void);
void *getMode(void *);
  
int main(void) {

  int gripperAngle, handAngle, handAngle1, handAngle2, handAngle3, newHandAngle;
  pthread_t tid;

  printf("Opening the device driver...\n");
   
  fp = open("/dev/Robot", O_RDWR);

  if (fp < 0) {
    printf("Device driver failed to open.  Error code = %d\n", fp);
    return -1;
  }
  else {

    printf("Device driver opened succesfully\n");

    // Linux doesn't provide a means to determine if a key has been pressed
    // and we can't use getchar() in the main processing loop because it blocks
    // so we'll create a separate thread to handle user input from the keyboard.
    // That thread will execute the getMode() function below
    pthread_create(&tid, NULL, getMode, (void *) NULL);

    gripperAngle = 0;
    handAngle    = 0;
    handAngle1   = 0;
    handAngle2   = 0;
    handAngle3   = 0;
    newHandAngle = 0;

    mode = 0;

    while (1) {

      // gripper arm test mode
      if (mode == 1) {
        setGripperAngle(0);
        sleep(1);
        setGripperAngle(100);
        sleep(1);
      }

      // open gripper arm 10 more degrees
      else if (mode == 2) {
        gripperAngle += 10;
        if (gripperAngle > 100) gripperAngle = 100;
        setGripperAngle(gripperAngle);
        mode = 0;
      }

      // close gripper arm 10 more degrees
      else if (mode == 3) {
        gripperAngle -= 10;
        if (gripperAngle < 0) gripperAngle = 0;
        setGripperAngle(gripperAngle);
        mode = 0;
        }

      // enter virtual reality mode
      else if (mode == 4) {
    	
    	  // get the current flex sensor reading
        handAngle = getHandAngle();

        //average together the 4 most recent hand angle readings to eliminate effects of shakey hand movement
        newHandAngle = (handAngle + handAngle1 + handAngle2 + handAngle3) / 4;
        handAngle3 = handAngle2;
        handAngle2 = handAngle1;
        handAngle1 = handAngle;

        //if hand angle has changed at least 5%, update robot gripper angle
        if ( (abs(newHandAngle - gripperAngle) > 4) || (gripperAngle < 10) || (gripperAngle > 90) ) {
          gripperAngle = newHandAngle;
          setGripperAngle(gripperAngle);
          printf("Gripper angle =%3d\r", gripperAngle); fflush(stdout);

        }
      }

      // exit the program
      else if (mode == 5) {
        close(fp);
        return 0;
      } 
    }
  }       
}

// write to the device driver to set the robot hand angle
void setGripperAngle(int gripperAngle) {

  unsigned char buffer[16];

  buffer[0] = (char) gripperAngle;
  write(fp, buffer, 1);
}

// read from the device driver to get the human hand angle
int getHandAngle(void) {

  unsigned char buffer[16];

  read(fp, buffer, 1);
  return (int) buffer[0]; 
}

// this function is executed by the thread that watches for user input from the keyboard
void *getMode(void * vargp) {

  char c;
  static struct termios oldt, newt;

  // set up to return keystrokes immediately rather than waiting for carriage return
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;
  newt.c_lflag &= ~(ICANON);
  tcsetattr(STDIN_FILENO, TCSANOW, &newt);

  printf("\n\n");
  printf("****************************************\n");
  printf("*                                      *\n");
  printf("*  The Linux Robot Driver              *\n");
  printf("*                                      *\n");
  printf("*  Menu:                               *\n");
  printf("*                                      *\n");
  printf("*    'T' : Test Gripper Arm            *\n");
  printf("*    '+' : Open  10 more degrees       *\n");
  printf("*    '-' : Close 10 more degrees       *\n");
  printf("*    'V' : Virtual Reality Modes       *\n");
  printf("*    'X' : Exit Program                *\n");
  printf("*                                      *\n");
  printf("****************************************\n");
  printf("\n\n");

  while (1) {
  	
  	// wait for the user to make a menu selection from the keyboard
    c = getchar();
    putchar('\b');
 
    // set the mode variable based on user's menu selection
    if ((c == 'T') || (c == 't'))
      mode = 1;
    else if (c == '+')
      mode = 2;
    else if (c == '-')
      mode = 3;
    else if ((c == 'V') || (c == 'v'))
      mode = 4;
    else if ((c == 'X') || (c == 'x')) {
      mode = 5;
      // on exit, restore the keyboard state that we changed above
      tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    }
  }
}
    
