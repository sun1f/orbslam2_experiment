#include <iostream>
#include <string>
#include <fstream>

#include <vector>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>
using namespace std;

class Node
{
public:
    /*Node()
    {

    }*/

    Node(int index, double x, double y, double theta)
    {
        this->index = index;
        this->x = x;
        this->y = y;
        this->theta = theta;
        p = new double[3];
        p[0] = x;
        p[1] = y;
        p[2] = theta;
    }

    double x, y, theta;
    int index;
    double *p;
};

class ReadG2O
{
public:
    ReadG2O(const string &fName)
    {
        // Read the file in g2o format
        fstream fp;
        fp.open(fName.c_str(), ios::in);

        string line;
        int v = 0;
        int e = 0;
        while (std::getline(fp, line))
        {
            vector<string> words;
            boost::split(words, line, boost::is_any_of(" "), boost::token_compress_on);
            if (words[0].compare("VERTEX_SE2") == 0)
            {
                v++;
                int node_index = boost::lexical_cast<int>(words[1]);
                double x = boost::lexical_cast<double>(words[2]);
                double y = boost::lexical_cast<double>(words[3]);
                double theta = boost::lexical_cast<double>(words[4]);

                Node *node = new Node(node_index, x, y, theta);
                nNodes.push_back(node);
            }
        }
    }

    // write nodes to file to be visualized with python script
    void writePoseGraph(const string &fname)
    {
        cout << "writePoseGraph : " << fname << endl;
        fstream fp;
        fp.open(fname.c_str(), ios::out);
        for (int i = 0; i < this->nNodes.size(); i++)
        {
            fp << nNodes[i]->index << " " << nNodes[i]->p[0] << " " << nNodes[i]->p[1] << " " << nNodes[i]->p[2] << endl;
        }
        // fp << "hello\n";
        // fp << "hello\n";
    }

    // private:
    vector<Node *> nNodes; // storage for node
};

int main(int argc, char **argv)
{
    if (argc != 3)
        cerr << endl
             << "./convert_g2o file_before file_after" << endl;
    string filename1 = argv[1];
    ReadG2O g1(filename1);
    g1.writePoseGraph("../result/txt/2D_before_opt.txt");

    string filename2 = argv[2];
    ReadG2O g2(filename2);
    g2.writePoseGraph("../result/txt/2D_after_GN.txt");
    string filename3 = argv[3];
    ReadG2O g3(filename3);
    g3.writePoseGraph("../result/txt/2D_after_LM.txt");
    string filename4 = argv[4];
    ReadG2O g4(filename4);
    g4.writePoseGraph("../result/txt/2D_after_DL.txt");

    return 0;
}