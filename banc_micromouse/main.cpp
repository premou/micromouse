#include <iostream>
#include <sstream>
#include <fstream>
#include<windows.h>
#include "serial.h"
#include "attribute.h"

using namespace std;

Serial * com_robot = 0;
Serial * com_table = 0;

int main_menu()
{
    int selection = 0;
    bool select_loop = true;
    while(select_loop)
    {
        cout << "Select sensor : " << endl;
        cout << "DL        - 1" << endl;
        cout << "FL+FR     - 2" << endl;
        cout << "DR        - 3" << endl;
        cout << "Save&Quit - 0" << endl;
        cout << ">";
        char c;
        cin >> c;
        switch(c)
        {
        case '1':
            {
                cout << "Calibrating DL..." << endl;
                selection = 1;
                select_loop = false;
            }
            break;
        case '2':
            {
                cout << "Calibrating FL+FR..." << endl;
                selection = 2;
                select_loop = false;
            }
            break;
        case '3':
            {
                cout << "Calibrating DR..." << endl;
                selection = 3;
                select_loop = false;
            }
            break;
        case '0':
            {
                cout << "Save & Quit!" << endl;
                select_loop = false;
            }
            break;
        default:
            {

            }
            break;
        }
    }
    return selection;
}

void zero_machine()
{
    cout << "Do zero machine on table :" << endl;
    cout << "Zero & Continue - 0" << endl;
    cout << ">";
    char c;
    cin >> c;
    com_table->WriteData("0\n",2);
}

int data[4][250];
t_attribute_manager att;
t_attribute dlsensor(1);
t_attribute drsensor(4);
t_attribute flsensor(2);
t_attribute frsensor(3);

char table_recv_buffer[4096];
char * table_recv_position = table_recv_buffer;

char robot_recv_buffer[12000];
char * robot_recv_position = robot_recv_buffer;

void fill(char * begin, char * end, char pattern )
{
    for(;begin<end;++begin)
    {
        *begin=pattern;
    }
}

char * copy(char const * begin, char const * end, char * begin2 )
{
    while(begin<end)
    {
        *begin2++=*begin++;
    }
    return begin2;
}

// Receive decoder
char const * unmarshalling_com_table(
    char const * begin,
    char const * end
    )
{
    char const * eom = begin;
    while(*eom!='\n' && eom<end) ++eom;
    if(eom!=end)
    {
        return eom+1;
    }
    else
        return begin;
}

char const * unmarshalling_com_robot(
    char const * begin,
    char const * end,
    char & command,
    char const * &arguments
    )
{
    char const * som = begin;
    while(*som!='@' && som<end) ++som;
    char const * eom = som+1;
    while(*eom!='#' && eom<end) ++eom;
    if(som!=end && eom!=end)
    {
        command = *(som+1);
        arguments = (som+2);
        return eom;
    }
    else
        return begin;
}

void flush_com_table()
{
    table_recv_position = table_recv_buffer;
    while( com_table->ReadData(table_recv_position,sizeof(table_recv_buffer)) != -1 );
    fill(table_recv_buffer,table_recv_buffer+sizeof(table_recv_buffer),0x00);
}

void flush_com_robot()
{
    robot_recv_position = robot_recv_buffer;
    while( com_robot->ReadData(robot_recv_position,sizeof(robot_recv_buffer)) != -1 );
    fill(robot_recv_buffer,robot_recv_buffer+sizeof(robot_recv_buffer),0x00);
}

unsigned int const max_distance = 180;

void calibrate(int selection)
{
    flush_com_table();
    flush_com_robot();
    unsigned int distance = 0;
    unsigned int old_distance = 0;
    bool record = false;
    com_table->WriteData("B\n",2);
    while(distance<max_distance)
    {
// Receive & Decode
	    for(;;)
        {
            int len = com_robot->ReadData(robot_recv_position,robot_recv_buffer+sizeof(robot_recv_buffer)-robot_recv_position);
            if(len!=-1)
            {
                robot_recv_position+=len;
            }
            char const * end = unmarshalling_com_table(
                robot_recv_buffer,
                robot_recv_position
            );
            if( end != robot_recv_buffer )
            {
                istringstream is(robot_recv_buffer);
                int fl, fr, dl, dr;
                is >> dl >> fl >> fr >> dr;
                att.set(1,dl);
                att.set(2,fl);
                att.set(3,fr);
                att.set(4,dr);

                // remove decoded command
                char * end2 = copy(end,robot_recv_buffer+sizeof(robot_recv_buffer),robot_recv_buffer);
                fill(end2,robot_recv_buffer+sizeof(robot_recv_buffer),0x00);
                robot_recv_position -= (end-robot_recv_buffer);
            }
            else
                break;
        }
        {
            int len = com_table->ReadData(table_recv_position,table_recv_buffer+sizeof(table_recv_buffer)-table_recv_position);
            if(len!=-1)
            {
                table_recv_position+=len;
            }
            char const * end = unmarshalling_com_table(
                table_recv_buffer,
                table_recv_position
            );
            if( end != table_recv_buffer )
            {
                //cout << table_recv_buffer << end;
                distance = atoi(table_recv_buffer);
                if(distance!=old_distance)
                {
                    old_distance = distance;

                    record = true;
                }
                // remove decoded command
                char * end2 = copy(end,table_recv_buffer+sizeof(table_recv_buffer),table_recv_buffer);
                fill(end2,table_recv_buffer+sizeof(table_recv_buffer),0x00);
                table_recv_position = table_recv_buffer;
            }
        }
        if(record)
        {
            cout << "d=" << distance << "mm";
            record=false;
            switch(selection)
            {
            case 1:
                data[0][distance]=dlsensor.get();
                cout << " adc=" << data[0][distance] << endl;
                break;
            case 2:
                data[1][distance]=flsensor.get();
                data[2][distance]=frsensor.get();
                cout << " adc=" << data[1][distance];
                cout << " adc=" << data[2][distance] << endl;
                break;
            case 3:
                data[3][distance]=drsensor.get();
                cout << " adc=" << data[3][distance] << endl;
                break;
            }
        }
    }
    com_table->WriteData("S\n",2);
    Sleep(1000);
    com_table->WriteData("O\n",2);
}

void save_data_file()
{
    ofstream of("data.txt");
    for(size_t distance=0; distance<max_distance; ++distance)
    {
        of << distance << " ";
        for(size_t index=0; index<4; ++index)
        {
            of << data[index][distance] << " ";
        }
        of << "\n";
    }
}

int main()
{
    cout << "Open robot com...";
    //com_robot = new Serial("\\\\.\\COM6",115200);
    com_robot = new Serial("\\\\.\\COM4",115200);
    if (com_robot->IsConnected())
    {
        cout << " [OK]" << endl;
    }
    else
    {
        cout << " [FAIL]" << endl;
    }
    cout << "Open table com...";
    com_table = new Serial("\\\\.\\COM7",9600); //38400);
    //com_table = new Serial("\\\\.\\COM25",9600); //38400);
    if (com_table->IsConnected())
    {
        cout << " [OK]" << endl;
    }
    else
    {
        cout << " [FAIL]" << endl;
    }

    att.add(&dlsensor,1);
    att.add(&flsensor,2);
    att.add(&frsensor,3);
    att.add(&drsensor,4);

    save_data_file();

    for(;;)
    {
        int selection = main_menu();
        if(selection==0)
        {
            save_data_file();
            break;
        }
        else
        {
            zero_machine();
            calibrate(selection);
            save_data_file();
        }
    }
    return 0;
}
