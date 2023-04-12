#include<iostream>
#include<fstream>

using namespace std;

int main() {
    ifstream fin;
    fin.open("./KeyFrameTrajectory.txt");
    string s;
    while(getline(fin, s)) {
        cout << "read from file" << s << endl;
    }




    fin.close();
}

