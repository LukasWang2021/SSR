#include <stdlib.h>
#include <iostream>
#include <string.h>
#include <fstream>
#include <vector>

using namespace std;

int main(int argc, char** argv)
{
    printf("begin\n");
    vector<string> v;
    ifstream infile("matlab.txt");//定义文件流对象
    ofstream outfile("output.txt");
    string line;

    while(getline(infile, line))
    {
        if (line.find('=') == line.npos)
        {
            //cout<<"NONE"<<endl;
            continue;
        }
        line.erase(0,line.find_first_not_of(" "));
        line.erase(line.find_first_of("="));
        line.erase(line.find_last_not_of(" ") + 1);

        v.push_back(line);
    }

    int num = 10;
    if (argc > 1)
    {
        num = atoi(argv[1]);
    }
    for (int i = 0; i < v.size(); ++i)
    {
        if (i % num == 0)
        {
            outfile<<"    double ";
            cout<<"    double ";
        }
        if ((i + 1)%num == 0)
        {
            outfile<<v[i]<<";"<<endl;
            cout<<v[i]<<";"<<endl;
            continue;
        }
        if (i + 1 == v.size())
        {
            outfile<<v[i]<<";"<<endl;
            cout<<v[i]<<";"<<endl;
            continue;
        }
        outfile<<v[i]<<",";
        cout<<v[i]<<",";
    }

    infile.close();
    outfile.close();
    printf("\nend\n");
    return 0;
}

