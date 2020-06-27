#ifndef GNUPLOT_H
#define GNUPLOT_H

#include <iostream>

using namespace std;

class gnuplot{
    public: 
        gnuplot(){
            gnuplotpipe = popen("gnuplot -persist", "w"); //using the popen library fuction to open gnuplot
            if (!gnuplotpipe){
                cerr<<("Gnuplot not found !");
            }
        }

        ~gnuplot(){
            fprintf(gnuplotpipe, "exit\n");
            pclose(gnuplotpipe);
        }
        void operator () (const string & command){
            fprintf(gnuplotpipe, "%s\n", command.c_str());
            fflush(gnuplotpipe);
        }

    void graph_file(string name_file_data, string x_label, string y_label){
        this->operator()("set term png");
        this->operator()("set output \"" +name_file_data+ ".png\"");
        this->operator()("set xlabel \""+ x_label +"\"");
        this->operator()("set ylabel \""+ y_label +"\"");
        //this->operator()("set yrange [0:] ");
        this->operator()("set xrange [0:] ");
        this->operator()("set style line 10 linetype 2 lc rgb \'blue\' pointtype 7");
        this->operator()("plot \""+name_file_data+".dat\" u 1:2 notitle with lp ls 10");
    }
        
    protected:
        FILE* gnuplotpipe;
        
};
#endif