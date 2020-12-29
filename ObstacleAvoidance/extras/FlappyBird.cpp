#include <iostream> //created by HAKERIS1010
#include <cstdlib>
#include <windows.h>
#include <conio.h>
#include <ctime>
#include <fstream>
#include <iomanip>

using namespace std;

ifstream inp;  //data file reading and writing operators
ofstream outp;

char c[30][21]; //variable for storing screen particles (pixels)
int n[30][21];  //variable for checking
int highscore;
int contr,tuk=0,score=0,t=0,bt=0,birdx=0,birdy=0; //variaous variables for certain operations
bool err; //boolean for error detection

void game();  //various functions
void screen();
void pipes();
void bird();
bool gameover();
void checkscore();
void help();
void menu();
void endgame();
void credits();

int main()
{
srand(time(0));  //seeding random number gen, we will need it later;
inp.open("/Program Files/FlappyBird/options.txt");  //opening file in which highscore is stored
if(inp.is_open()) //if file opens successfully, it reads the highscore
{
    inp>>highscore;
    inp.close();
    err=false;  //error will be false, because file opened successfully
}
else
{
    highscore=0; //if file doesnt exist, highscore will be 0, and err will be true
    err=true;
}

int a=0,b;
char sl; //selection variable
while(1) //loop for repeating actions after each start
{
    if(a==0) goto play; 
    if(a>0)               //if you play not the first time, it will ask you if you want to play
    {
        score=0;
        cout<<"Do you want to play again? [y/n] ";
        cin>>sl;
        if(sl=='n') goto quit;
        else goto play;
    }
    play:
    menu(); //calling menu function
    cin>>sl;
    switch(sl) //menu selections
    {
        case '1':
        {
            game(); //if you choose play, it calls function game
            break;
        }
        case '2': //other selections-other functions
        {
            help(); 
            goto play;
            break;
        }
        case '3':
        {
            credits();
            goto play;
            break;
        }
        case '4':
        {
            goto quit; //exits game
            break;
        }
        default:
        {
            goto play;
            break;
        }
    }
    a++; //variable for checking how many times you've played
}
quit:
{
   cout<<"I quit."; //stops game, app closes.
}

return 0;
}

void game()  //function for playing game
{
    int x,y;
    char s;
    for(y=0;y<21;y++)  //setting screen
    {
        for(x=0;x<30;x++)
        {
            if(y<20)
            {
            c[x][y]=' ';
            n[x][y]=0;
            }
            if(y==20)
            {
                c[x][y]='-';
                n[x][y]=2;
            }
        }
    }
    c[10][10]='*';  //in these coordinates will be our bird, marked *
    screen();      //calls func for showin screen
    while(1)       //loop starts, actual gameplay begins
    {
        s='~';  //default control variable
        Sleep(0.2*1000);  //this sets how fast everyhing moves
        t++;              //this is a variable for storing 'time',or how many times a loop passed
        if(kbhit()) //if key is pressed, certain operations are done for bird to move up.
        {
            s=getch();        //gets what key is pressed
            if(s!='~') tuk=1; //if it is not default, then 'tuk' will be equal to 1, meaning that bird will fly up
        }
        for(x=0;x<30;x++) //just setting ground
        {
            c[x][20]='-';
            n[x][20]=2;
        }
        bird();                       //cals bird move function
        checkscore();                 //checks score
        if(gameover()==true) goto gameEnd;   //checks if bird hits pipes, if yes, game ends
        pipes();                             //spawns and moves pipes
        if(score>highscore) highscore=score;  //i think this is clear
        screen();                            //finally, calls screen function to show enerything.

        if(tuk>0) tuk++;           //if key is pressed, bird will fly up 2 times.
        if(tuk==3) tuk=0;          //after that, bird falls
    }
    gameEnd:   //ends game
    {
        if(score>highscore) highscore=score;
        if(err==false)              //if hi-score file exists, it writes your new highscore there.
        {
            outp.open("/Program Files/FlappyBird/options.txt");
            outp<<highscore;
            outp.close();
        }
        screen();    //shows ending screen, and returns to int main
        endgame();
        return;
    }
}

void screen()    //func for showing screen
{
    int x,y;
    system("cls");    //clears console
    for(y=0;y<21;y++) //shows pixels on their coordinates, and your score
    {
        for(x=0;x<30;x++)
        {
            if(x<29) cout<<c[x][y];
            if(x==29) cout<<c[x][y]<<endl;
        }
    }
    cout<<""<<endl;
    cout<<"Your Score: "<<score;
}

void pipes()  //pipe movement and spawn func
{
    int x,y,r;
    if(t==10)   //if time is 10, or loop has passed 10 times, it spawns a new pipe
    {
        r=(rand()%11)+5;  //generates random number, which will be the pipe's hole center
        for(y=0;y<20;y++)  // only y coordinate is needed
        {
            c[29][y]='|';  //sets pipe
            n[29][y]=2;    //n will be 2, for checking if bird has hit it,
        }
        c[29][r-1]=' ';  //sets hole
        c[29][r]=' ';
        c[29][r+1]=' ';
        n[29][r-1]=0;
        n[29][r]=0;
        n[29][r+1]=0;
        t=0;
        goto mv; //moves pipes
    }
    else goto mv;
    mv:                 //pipe movement
    {
        for(y=0;y<20;y++)  //loops for generating coordinates
        {
            for(x=0;x<30;x++)
            {
                if(c[x][y]=='|')  //all the pipes will be moved left by 1;
                {
                    if(x>0)
                    {
                        c[x-1][y]='|';
                        n[x-1][y]=2;
                        c[x][y]=' ';
                        n[x][y]=0;
                    } 
                    if(x==0)  //if screen ends (x=0) pipe will dissapear, to prevent errors
                    {
                        c[x][y]=' ';
                        n[x][y]=0;
                    }
                }
            }
        }
    }
}

void bird()   //bird movement function!
{
    int x,y;
    if(tuk>0) //if key is pressed, bird moves up
    {
        bt=0;
        for(y=0;y<20;y++)   //loops for finding bird coordinates
        {
            for(x=0;x<30;x++)
            {
                if(c[x][y]=='*')
                {
                    if(y>0)
                    {
                    c[x][y-1]='*';  //bird moves up by 1;
                    c[x][y]=' ';
                    birdx=x;        //sets bird x coordinate
                    birdy=y-1;      //sets bird y coord
                    return;         //retuns to game func
                    }
                }
            }
        }
    }
    else   //if no key is pressed, bird falls
    {
        bt++;
        for(y=0;y<20;y++)
        {
            for(x=0;x<30;x++)
            {
                if(c[x][y]=='*')
                {
                    if(y<20)  //if bird is not on the ground
                    {
                        if(bt<3)   //if bird time is lower that 3, it falls 1 pixel
                        {
                            c[x][y+1]='*';
                            c[x][y]=' ';
                            birdx=x;
                            birdy=y+1;
                            return;
                        }
                        else if(bt>2 && bt<5)  //more time has passed, faster bird falls (acceleration)
                        {
                            c[x][y+2]='*';
                            c[x][y]=' ';
                            birdx=x;
                            birdy=y+2;
                            return;
                        }
                        else if(bt>4)
                        {
                            c[x][y+3]='*';
                            c[x][y]=' ';
                            birdx=x;
                            birdy=y+3;
                            return;
                        }
                    }
                    else
                    {
                        return;  //if bird is already on the ground, function returns to check for game over.
                    }
                }
            }
        }
    }
}
void checkscore()  //checks if bird gained score
{
    int y;
    for(y=0;y<20;y++)
    {
        if(c[birdx][y]=='|')  //if bird x coord is equal to pipe's coord, you get 1 point
        {
            score++;
            return;
        }
    }
}

bool gameover()  //checks if bird has hit something
{
    int x,y,f=0;
    if(birdy>19) //checks if bird hits ground
    {
        c[birdx][19]='*';  //sets bird and ground again, prevents errors
        c[birdx][20]='-';
        f=1;           //f=1, means function will return true
        goto quit;
    }
    else
    {     //checks if bird has hit pipes, here the 'n' variable is needed (pipe's coordinate's n is equal 2 (more than 0))
        if(n[birdx][birdy]>0 && (c[birdx][birdy]=='|' || c[birdx][birdy]=='*'))
        {
            c[birdx][birdy]='|';
            c[birdx-1][19]='*';
            f=1;
            goto quit;
        }
    }
    quit:
    if(f==1) return true;
    else return false;
}

void endgame() //just some screens for certain actions
{
    screen();   //this one pops up if game ends
    cout<<""<<endl<<endl;
    cout<<" ------------------------------------------------------------------------- "<<endl;
    cout<<"|    *****      *     *       * ******       ****  *       ****** ****    |"<<endl;
    cout<<"|   *          * *    * *   * * *           *    *  *     * *     *   *   |"<<endl;
    cout<<"|   *  ****   *   *   *  * *  * *****       *    *   *   *  ****  ****    |"<<endl;
    cout<<"|   *  *  *  *******  *   *   * *           *    *    * *   *     * *     |"<<endl;
    cout<<"|    *****  *       * *       * ******       ****      *    ***** *   *   |"<<endl;
    cout<<" ------------------------------------------------------------------------- "<<endl;
    cout<<""<<endl<<endl;
    cout<<"                        Y O U R   S C O R E : "<<score<<endl<<endl;
    cout<<"                        H I G H   S C O R E : "<<highscore<<endl;
    cout<<""<<endl<<endl;
}

void menu()  //shows menu
{
    system("cls");
    cout<<""<<endl;
    cout<<" --------------------------------------------------------  "<<endl;
    cout<<"|                                                        | "<<endl;
    cout<<"|   **** *    **** **** **** *   *    ***  * ***  ***    | "<<endl;
    cout<<"|   *    *    *  * *  * *  * *   *    *  * * *  * *  *   | "<<endl;
    cout<<"|   ***  *    **** **** **** *****    ***  * ***  *  *   | "<<endl;
    cout<<"|   *    *    *  * *    *      *      *  * * *  * *  *   | "<<endl;
    cout<<"|   *    **** *  * *    *      *      ***  * *  * ***    | "<<endl;
    cout<<"|                                                  v 1.0 | "<<endl;
    cout<<" --------------------------------------------------------  "<<endl;
    cout<<""<<endl<<endl;
    cout<<"                  High Score:  "<<highscore<<endl<<endl;
    cout<<""<<endl<<endl;
    cout<<"                     M E N U:    "<<endl<<endl;
    cout<<"                  1: Start Game  "<<endl<<endl;
    cout<<"                  2: Help        "<<endl<<endl;
    cout<<"                  3: Credits     "<<endl<<endl;
    cout<<"                  4: Exit        "<<endl<<endl;
}

void credits()
{
    char sel;
    system("cls");
    while(true)
    {
    cout<<""<<endl<<endl;
    cout<<"               Lead programmer: hakeris1010 "<<endl<<endl;
    cout<<"               Designer: hakeris1010 "<<endl<<endl;
    cout<<"               Testers: hakeris1010 "<<endl<<endl;
    cout<<"               Special thanks to: hakeris1010 ,Dong Nguyen (original)"<<endl<<endl;
    cout<<"               Version: 1.0 "<<endl<<endl<<endl;
    cout<<"Go back? [y/n]  ";
    cin>>sel;
    if(sel=='y') return;
    else system("cls");
    }
}

void help()
{
    char sel;
    system("cls");
    while(true)
    {
    cout<<""<<endl<<endl;
    cout<<"                   Controls: Press any key to fly up.         "<<endl<<endl;
    cout<<"             Goal: Fly through the holes between the pipes.   "<<endl;
    cout<<"             When you pass through the hole,you get 1 point.  "<<endl;
    cout<<"                    Try to pass as much as you can.           "<<endl;
    cout<<"            But be careful, don't hit the pipes or the ground!"<<endl<<endl;
    cout<<"                          Are you ready? Go!                  "<<endl<<endl<<endl;
    cout<<"Go back? [y/n]  ";
    cin>>sel;
    if(sel=='y') return;
    else system("cls");
    }
}
//version 1.0
//made by hakeris1010
//thanks for playing!!! 