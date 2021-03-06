/* This file is part of COVISE.
 *

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */
#pragma once
#include <openGA.hpp>
#include <Cam.h>
#define CAMS_PER_POINT 16
#define CAMS 64
#define CAM_POINTS 4
//8:384 , 9:423 ,current: 192
#if(1)
class GA
{
public:
    GA(std::vector<Cam*>& cam, std::vector<SafetyZone*>& safetyZoneList);
    ~GA()=default;
    std::vector<int> getfinalCamPos() const;

private:
    std::ofstream output_file;              //store result of GA
    std::vector<Cam*>& camlist;
    std::vector<int> priorityList;
    size_t nbrpoints;                 //number of points to observe
    const size_t nbrcams=camlist.size();    //number of cameras

    struct MySolution{
        //https://stackoverflow.com/questions/40887305/c-initializing-a-vector-inside-a-struct-definition
         std::vector<int> cam = std::vector<int>(CAMS, 0);
         std::string to_string() const
         {

             std::string myString;
             int cnt =1;
             for(auto i : cam)
             {
                myString += "cam"+std::to_string(cnt)+":"+std::to_string(i)+" ";
                cnt++;
             }
             return
                 std::string("{") + myString + "}";

         }

     };
    struct MyMiddleCost{double objective;};
    typedef EA::Genetic<MySolution,MyMiddleCost> GA_Type;
    typedef EA::GenerationType<MySolution,MyMiddleCost> Generation_Type;
    GA_Type ga_obj;
    int myrandom();
    int myrandom2();
    void init_genes(MySolution& p,const std::function<double(void)> &rnd01);
    MySolution mutate(const MySolution& X_base,const std::function<double(void)> &rnd01,double shrink_scale);
    MySolution crossover(const MySolution& X1, const MySolution& X2,const std::function<double(void)> &rnd01);
    bool eval_solution(const MySolution& p,MyMiddleCost &c);
    double calculate_SO_total_fitness(const GA_Type::thisChromosomeType &X);
    void SO_report_generation(int generation_number,const EA::GenerationType<MySolution,MyMiddleCost> &last_generation,const MySolution& best_genes);

};
#else
class GA
{
public:
    GA(std::vector<Cam*>& cam, std::vector<SafetyZone*>& safetyZoneList);
    ~GA()=default;
    std::vector<std::array<int,16>> getfinalCamPos() const;

private:
    std::ofstream output_file;              //store result of GA
    std::vector<Cam*>& camlist;
    std::vector<int> priorityList;
    size_t nbrpoints;                 //number of points to observe
    const size_t nbrcams=camlist.size();    //number of cameras
    double prioParam =2;
    static std::array<int,CAMS_PER_POINT> camPoints;

    struct MySolution{
        //https://stackoverflow.com/questions/40887305/c-initializing-a-vector-inside-a-struct-definition
         //std::vector<std::array<int,16>> cam = std::vector<std::array<int>>(CAMS, 0);
        std::vector<std::array<int,16>> cam = std::vector<std::array<int,16>>(CAM_POINTS,camPoints);

       // std::vector<std::array<int,16>> cam;
       // cam.resize(CAM_POINTS,camPoints);
    public:

         std::string to_string() const
         {

             std::string myString;
             int cnt =1;
             for(const auto& i : cam)
             {
                 for(const auto& x :i)
                 {
                     myString += "cam"+std::to_string(cnt)+":"+std::to_string(x)+" ";
                     cnt++;
                 }
             }
             return std::string("{") + myString + "}";
         }

     };
    struct MyMiddleCost{double objective;
                        int prio1 = 0;
                        int prio2 = 0;
                       };
    typedef EA::Genetic<MySolution,MyMiddleCost> GA_Type;
    typedef EA::GenerationType<MySolution,MyMiddleCost> Generation_Type;
    GA_Type ga_obj;
    int myrandom();
    int myrandom1(int maxNbr);
    void init_genes(MySolution& p,const std::function<double(void)> &rnd01);
    MySolution mutate(const MySolution& X_base,const std::function<double(void)> &rnd01,double shrink_scale);
    MySolution crossover(const MySolution& X1, const MySolution& X2,const std::function<double(void)> &rnd01);
    bool eval_solution(const MySolution& p,MyMiddleCost &c);
    double calculate_SO_total_fitness(const GA_Type::thisChromosomeType &X);
    void SO_report_generation(int generation_number,const EA::GenerationType<MySolution,MyMiddleCost> &last_generation,const MySolution& best_genes);

    void slice(std::vector<std::array<int,16>> &newVec,const std::vector<std::array<int,16>> &v, int m, int n)
    {
        std::copy(v.begin() + m, v.begin() + n + 1, newVec.begin()+m);
    }
};
#endif
