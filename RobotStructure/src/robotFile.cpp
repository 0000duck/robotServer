#include "robotFile.h"
#include <iostream>
#include <fstream>

using namespace std;

int copy_file(const char* src, const char* dst){
    ifstream in;
    ofstream out;

    in.open(src, std::ios::binary);
    out.open(dst, std::ios::binary);

    if(in.fail()){
        std::cout << "Error: fail to open the source file!" << std::endl;
        in.close();
        out.close();
        return 1;
    }
    if(out.fail()){
        std::cout << "Error: fail to create the destination file." << std::endl;
        in.close();
        out.close();
        return 1;
    }

    out << in.rdbuf();
    in.close();
    out.close();
    return 0;
}

bool get_line_string(ifstream& p_fin, vector<string>& p_s){
    p_s.clear();
    if(p_fin.eof()){
        return false;
    }
    else{
        string s;
        getline(p_fin, s);

        while(!s.empty()){
            string tmp;
            int index = s.find(" ");
            if(index == -1){
                index = s.find("\n");
                if(index = -1){
                    tmp = s;
                    p_s.push_back(tmp);
                    break;
                }
                else{
                    tmp = s.substr(0, index);
                    if(!tmp.empty())
                        p_s.push_back(tmp);
                    s = s.substr(index+1);
                }
            }
            else{
                tmp = s.substr(0, index);
                if(!tmp.empty())
                    p_s.push_back(tmp);
                s = s.substr(index+1);
            }
        }

        return true;
    }
}
