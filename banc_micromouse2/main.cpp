#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <map>
using namespace std;

struct data_type
{
    unsigned int dist;
    unsigned int dl;
    unsigned int fl;
    unsigned int fr;
    unsigned int dr;
    friend ostream & operator << (ostream &out, data_type const & d);
    friend istream & operator >> (istream &in,  data_type & d);
};

ostream & operator << (ostream &out, data_type const & d)
{
    out << d.dist << "mm " << d.dl << " " << d.fl << " " << d.fl << " " << d.dr;
    return out;
}

istream & operator >> (istream &in, data_type & d)
{
    in >> d.dist >> d.dl >> d.fl >> d.fr >> d.dr;
    return in;
}

void invert_and_scale(vector<int> const & in, vector<int> & out)
{
    size_t position = 0; // 0mm
    for(int adc = 4095; adc>=0; --adc)
    {
        out[adc] = position;
        if(adc <= in[position] && position<in.size())
            position++;
        //cout << adc << "->" << out[adc] << endl;
    }
}

int main()
{
    ifstream input_data_file("data.txt");
    if(input_data_file)
    {
        vector<data_type> raw_data;
        /// read raw data
        string input_line;
        while(getline(input_data_file,input_line))
        {
            istringstream is(input_line);
            data_type entry;
            is >> entry;
            raw_data.push_back(entry);
            cout << entry << endl;
        };
        input_data_file.close();
        cout << "number of entries:" << raw_data.size();
        /// filter raw data
        vector<int> filtered_dl;
        vector<int> filtered_fl;
        vector<int> filtered_fr;
        vector<int> filtered_dr;
        for(auto d : raw_data )
        {
            filtered_dl.push_back(d.dl);
            filtered_fl.push_back(d.fl);
            filtered_fr.push_back(d.fr);
            filtered_dr.push_back(d.dr);
        }
        sort(filtered_dl.begin(),filtered_dl.end(),std::greater<int>());
        sort(filtered_fl.begin(),filtered_fl.end(),std::greater<int>());
        sort(filtered_fr.begin(),filtered_fr.end(),std::greater<int>());
        sort(filtered_dr.begin(),filtered_dr.end(),std::greater<int>());
        /// invert table and full scale
        vector<int> table_dl(4096);
        vector<int> table_fl(4096);
        vector<int> table_fr(4096);
        vector<int> table_dr(4096);
        invert_and_scale(filtered_dl,table_dl);
        invert_and_scale(filtered_fl,table_fl);
        invert_and_scale(filtered_fr,table_fr);
        invert_and_scale(filtered_dr,table_dr);
        /// produce c code
        {
            ofstream output_file("wall_sensor_table.h");
            map< string,vector<int> > list = { {"table_dl",table_dl},{"table_fl",table_fl},{"table_fr",table_fr},{"table_dr",table_dr}};
            for( auto t : list )
            {
                output_file << "static uint32_t const " << t.first << "[4096]={\n";
                size_t adc_value = 0;
                size_t end_of_data = t.second.size()-1;
                for(auto x : t.second )
                {
                    output_file << "\t" << x;
                    if(adc_value!=end_of_data)
                        output_file << ",";
                    output_file << " // mm for adc value = " << adc_value++ << "\n";
                }
                output_file << "};\n" << endl;
            }
            output_file.close();
        }
    }
    else
        cout << "Erreur d'ouverture du fichie d'entree!" << endl;
    return 0;
}
