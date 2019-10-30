 /* This file is part of COVISE.

   You can use it under the terms of the GNU Lesser General Public License
   version 2.1 or later, see lgpl-2.1.txt.

 * License: LGPL 2+ */
#include <iostream>
#include <string>
#include <vector>
#include <numeric>
#include <fstream>
#include<cstdlib>
#include<ctime>
#include<algorithm>
#include <array>

#include "GA.hpp"

#define CAMS_PER_POINT 16
#define CAMS 64
#define CAM_POINTS 4
#if(1)
int GA:: myrandom() {
    if (rand() % 2 == 0)
        return 1;
    else return 0;
}

void GA::init_genes(MySolution& p,const std::function<double(void)> &rnd01)
{
   //  EA::Chronometer timer;
   //  timer.tic();
 //  p.cam.fill(0); // fill up with 0
 /*   int a = p.cam.size();
    for(size_t i=0;i<=p.cam.size()/CAMS_PER_POINT;i++)
    {
        const size_t count2 = i*CAMS_PER_POINT+myrandom2();
        p.cam[count2]=1;
    }
*/
     for(auto& i : p.cam)
       i = myrandom();

    //std::cout<<" init_genes "<<timer.toc()<<" seconds."<<std::endl;
}


bool GA::eval_solution(const MySolution& p,MyMiddleCost &c)
{
  //  EA::Chronometer timer;
  //  timer.tic();
    /*
    The heavy process of evaluation of solutions are assumed to be perfomed in eval function
    The result of this function is callde middle cost as it is not finalized.
    Constraint checking is also done here
    */
    //Allow only 1 cam per Camera Point
    /*for(size_t it =0; it <NUMBER_OF_CAMS - CAMS_PER_CAMPOINT; it+=CAMS_PER_CAMPOINT)
    {
        if(std::accumulate(p.cam.begin() + it, p.cam.begin() + it + NUMBER_OF_CAMS ,0) > 1);
            goto exit; // Fehler hier beim Durchiterieren ?
    }
    */

    c.objective = std::accumulate(p.cam.begin(), p.cam.end(),0);

    // 1. constraint: each observation Point must be observed with at least 1 camera
    //Loop over all Points in Visibility Matrix
    for(size_t it =0; it<nbrpoints; ++it )
    {
        int nbrOfCamsPerPoint =0;
        //For each Point go over each camera
        for(size_t it2 =0; it2<nbrcams; ++it2 )
           {
           // if(p.cam[it2]*camlist[it2]->visMat[it] != 0)
           //     nbrOfCamsPerPoint +=1;

                nbrOfCamsPerPoint += p.cam[it2]*camlist[it2]->visMat[it];
           }
        if(nbrOfCamsPerPoint < priorityList.at(it))//priorityList.at(it))//
            goto exit;

    }
    // at this point constraint 1 is fullfilled
//   std::cout<<" eval_solution "<<timer.toc()<<" seconds."<<std::endl;
    return true;
    exit:
         return false;

}

GA::MySolution GA:: mutate(const MySolution& X_base,const std::function<double(void)> &rnd01,double shrink_scale)
{
    EA::Chronometer timer;
    timer.tic();
    MySolution X_new;
    auto test=shrink_scale;
    bool in_range;
  /*  do{
        in_range=true;
      */  //X_new=X_base;

    /*   for(auto &i : X_new.cam)
        {
            i+=0.2*(rnd01()-rnd01())*shrink_scale;
            in_range=in_range&&(i>=0 && i<1);
        }
    */ /*   X_new.cam1+=0.2*(rnd01()-rnd01())*shrink_scale;
        in_range=in_range&&(X_new.cam1>=0 && X_new.cam1<1);
        X_new.cam2+=0.2*(rnd01()-rnd01())*shrink_scale;
        in_range=in_range&&(X_new.cam2>=0 && X_new.cam2<1);
        X_new.cam3+=0.2*(rnd01()-rnd01())*shrink_scale;
        in_range=in_range&&(X_new.cam3>=0 && X_new.cam3<1);
        X_new.cam4+=0.2*(rnd01()-rnd01())*shrink_scale;
        in_range=in_range&&(X_new.cam4>=0 && X_new.cam4<1);
    */
  // } while(!in_range);

 //   std::cout<<"mutate: "<<timer.toc()<<" seconds."<<std::endl;
  //  if(RANDOM_NUM < shrink_scale)
  //  {
        for(auto &i : X_new.cam)
            i = 1-i;
 //  }
    return X_new;
}


GA::MySolution GA::crossover(const MySolution& X1, const MySolution& X2,const std::function<double(void)> &rnd01)
{
    MySolution X_new;

    for(size_t it =0; it<X1.cam.size(); ++it )
   {
       // X_new.cam[it]=r*X1.cam[it]+(1.0-r)*X2.cam[it];
        X_new.cam[it]=X1.cam[it]*X2.cam[it];
      //  r=rnd01();
   }

   // std::for_each(X_new.cam.begin(),X_new.cam.end(),[](int &n){ n++; });

  /*  std::transform( X1.cam.begin(), X1.cam.end(),
                    X2.cam.begin(), X_new.cam.begin(),
                    std::multiplies<int>() ); // assumes values are 'int'
    */
    return X_new;
}

double GA::calculate_SO_total_fitness(const GA_Type::thisChromosomeType &X)
{
    // finalize the cost
    //obtain the final cost from the middle cost
    double final_cost=0;
    final_cost+=X.middle_costs.objective;
    return final_cost;
}



void GA::SO_report_generation(int generation_number,const EA::GenerationType<MySolution,MyMiddleCost> &last_generation,const MySolution& best_genes)
{
    std::cout
        <<"Generation ["<<generation_number<<"], "
        <<"Best="<<last_generation.best_total_cost<<", "
        <<"Average="<<last_generation.average_cost<<", "
        <<"Best genes=("<<best_genes.to_string()<<")"<<", "
        <<"Exe_time="<<last_generation.exe_time
        <<std::endl;

    output_file
        <<generation_number<<"\t"
        <<last_generation.average_cost<<"\t"
        <<last_generation.best_total_cost<<"\t"
        <<best_genes.to_string()<<"\n";
}

std::vector<int> GA::getfinalCamPos() const
{
   std::vector<int>result= ga_obj.last_generation.chromosomes.at(ga_obj.last_generation.best_chromosome_index).genes.cam;
   return result ;
}
GA::GA(std::vector<Cam*>& cam, std::vector<SafetyZone *> &safetyZoneList):camlist(cam)
{
    nbrpoints = safetyZoneList.size();

    for(const auto x : safetyZoneList)
        priorityList.push_back(x->getPriority());


    output_file.open("results.txt");
    output_file<<"step"<<"\t"<<"cost_avg"<<"\t"<<"cost_best"<<"\t"<<"solution_best"<<"\n";

    EA::Chronometer timer;
    timer.tic();

    using namespace std::placeholders;
    ga_obj.problem_mode=EA::GA_MODE::SOGA;
    ga_obj.multi_threading=true;
    ga_obj.idle_delay_us=10; // switch between threads quickly
    ga_obj.dynamic_threading=true;
    ga_obj.verbose=false;
    ga_obj.population=3000;
    ga_obj.generation_max=1000;
    ga_obj.calculate_SO_total_fitness=std::bind( &GA::calculate_SO_total_fitness, this, _1);
    ga_obj.init_genes=std::bind( &GA::init_genes, this, _1,_2);
    ga_obj.eval_solution=std::bind( &GA::eval_solution, this, _1,_2 );
    ga_obj.mutate=std::bind( &GA::mutate, this, _1,_2,_3 );
    ga_obj.crossover=std::bind( &GA::crossover, this, _1,_2,_3 );
    ga_obj.SO_report_generation=std::bind( &GA::SO_report_generation, this, _1,_2,_3 );
    ga_obj.crossover_fraction=0.7;
    ga_obj.mutation_rate=0.2;
    ga_obj.best_stall_max=10;
    ga_obj.elite_count=10;
    ga_obj.solve();

    std::cout<<"The problem is optimized in "<<timer.toc()<<" seconds.###########################################"<<std::endl;

    output_file.close();


}
#else

std::array<int,CAMS_PER_POINT>GA:: camPoints{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };

int GA:: myrandom() {
    if (rand() % 2 == 0)
        return 1;
    else return 0;
}

int GA:: myrandom1(int maxNbr) {

    return rand() % maxNbr ;
}


void GA::init_genes(MySolution& p,const std::function<double(void)> &rnd01)
{
    auto test = myrandom1(CAMS_PER_POINT);
    for(auto &x : p.cam)
        x.at(myrandom1(CAMS_PER_POINT)) =1;
}


bool GA::eval_solution(const MySolution& p,MyMiddleCost &c)
{

    //only 1 cam per cam position
 /*   for(const auto& x : p.cam)
    {
        if(std::accumulate(x.begin(),x.end(),0) > 1)
            goto exit;
    }
*/
    size_t cnt =0;
    std::vector<double> sumOfAllVisMats(nbrpoints);
    for(const auto& x : p.cam)
    {
        for(const auto& y:x)
        {
            if(y==1)
            {
                std::transform(sumOfAllVisMats.begin(), sumOfAllVisMats.end(), camlist[cnt]->getVisMat().begin(), sumOfAllVisMats.begin(), std::plus<double>());
            }
            cnt ++;
        }
    }

    size_t cnt2 =0;
    for(const auto& x :priorityList)
    {
        if(x==SafetyZone::PRIO1 && sumOfAllVisMats.at(cnt2)>= SafetyZone::PRIO1)//equal to PRIO1 zone ->2 cameras
        {
            c.prio1 ++;
        }
        else if(x==SafetyZone::PRIO2 && sumOfAllVisMats.at(cnt2)>= SafetyZone::PRIO2)
            c.prio2++;
        else //<-- ohne das hier optimiert er auch, aber ohne Garantie, dass alle Zonen abgedeckt werden!
        {
            std::cout<<"rejected"<<std::endl;
            goto exit;
        }
        cnt2++;
    }

    c.objective = -(prioParam * c.prio1 + c.prio2);


  /*  // 1. constraint: each observation Point must be observed with at least 1 camera
    //Loop over all Points in Visibility Matrix
    for(size_t it =0; it<nbrpoints; ++it )
    {
        int nbrOfCamsPerPoint =0;
        //For each Point go over each camera
        for(size_t it2 =0; it2<nbrcams; ++it2 )
        {
            if(priorityList[it] == 2)   //equal to PRIO1 zone ->2cameras
            {
                nbrOfCamsPerPoint += p.cam[it2]*camlist[it2]->visMat[it];
                if(nbrOfCamsPerPoint == priorityList[it])
                    c.prio1 +=1;
            }
            else if(priorityList[it] == 1) //equal to PRIO2 zone ->1cameras
            {
                nbrOfCamsPerPoint+= p.cam[it2]*camlist[it2]->visMat[it];
                if(nbrOfCamsPerPoint == priorityList[it])
                    c.prio2 +=1;
            }
        }
    //    if(nbrOfCamsPerPoint < priorityList.at(it))
      //      goto exit;

    }
*/
    return true;
   exit:
       return false;

}

GA::MySolution GA:: mutate(const MySolution& X_base,const std::function<double(void)> &rnd01,double shrink_scale)
{

    MySolution X_new;
    auto test=shrink_scale;
    bool in_range;

    for(auto &x : X_new.cam)
        x.at(myrandom1(CAMS_PER_POINT)) =1;
    return X_new;
}



GA::MySolution GA::crossover(const MySolution& X1, const MySolution& X2,const std::function<double(void)> &rnd01)
{
    int cutPoint =myrandom1(CAM_POINTS);
    MySolution X_new =X1;
  //  std::vector<std::array<int,16>> X_new;
   // X_new.resize(nbrCameras,arrayT);
  /*  for(size_t it =0; it<=cutPoint;++it)
    {
        X_new.cam.push_back(X1.cam.at(it));
    }
    for(size_t it =cutPoint+1; it<X1.cam.size();++it)
    {
        X_new.cam.push_back(X2.cam.at(it));
    }
*/
   //slice(X_new.cam,X1.cam, 0, cutPoint);
   //slice(X_new.cam,X2.cam,cutPoint,CAM_POINTS-1);
    return X_new;
}

double GA::calculate_SO_total_fitness(const GA_Type::thisChromosomeType &X)
{
    // finalize the cost
    //obtain the final cost from the middle cost
    double final_cost=0;
    final_cost+=X.middle_costs.objective;
    return final_cost;
}



void GA::SO_report_generation(int generation_number,const EA::GenerationType<MySolution,MyMiddleCost> &last_generation,const MySolution& best_genes)
{
    std::cout
        <<"Generation ["<<generation_number<<"], "
        <<"Best="<<last_generation.best_total_cost<<", "
        <<"Average="<<last_generation.average_cost<<", "
        <<"Best genes=("<<best_genes.to_string()<<")"<<", "
        <<"Exe_time="<<last_generation.exe_time
        <<std::endl;

    output_file
        <<generation_number<<"\t"
        <<last_generation.average_cost<<"\t"
        <<last_generation.best_total_cost<<"\t"
        <<best_genes.to_string()<<"\n";
}

std::vector<std::array<int,16>> GA::getfinalCamPos() const
{
   std::vector<std::array<int,16>>result= ga_obj.last_generation.chromosomes.at(ga_obj.last_generation.best_chromosome_index).genes.cam;
   return result ;
}
GA::GA(std::vector<Cam*>& cam, std::vector<SafetyZone *> &safetyZoneList):camlist(cam)
{
    nbrpoints = safetyZoneList.size();

    for(const auto x : safetyZoneList)
        priorityList.push_back(x->getPriority());


    output_file.open("results.txt");
    output_file<<"step"<<"\t"<<"cost_avg"<<"\t"<<"cost_best"<<"\t"<<"solution_best"<<"\n";

    EA::Chronometer timer;
    timer.tic();

    using namespace std::placeholders;
    ga_obj.problem_mode=EA::GA_MODE::SOGA;
    ga_obj.multi_threading=true;
    ga_obj.idle_delay_us=10; // switch between threads quickly
    ga_obj.dynamic_threading=true;
    ga_obj.verbose=true;
    ga_obj.population=1000;
    ga_obj.generation_max=1000;
    ga_obj.calculate_SO_total_fitness=std::bind( &GA::calculate_SO_total_fitness, this, _1);
    ga_obj.init_genes=std::bind( &GA::init_genes, this, _1,_2);
    ga_obj.eval_solution=std::bind( &GA::eval_solution, this, _1,_2 );
    ga_obj.mutate=std::bind( &GA::mutate, this, _1,_2,_3 );
    ga_obj.crossover=std::bind( &GA::crossover, this, _1,_2,_3 );
    ga_obj.SO_report_generation=std::bind( &GA::SO_report_generation, this, _1,_2,_3 );
    ga_obj.crossover_fraction=0.7;
    ga_obj.mutation_rate=0.3;
    ga_obj.best_stall_max=10;
    ga_obj.elite_count=10;
    ga_obj.solve();

    std::cout<<"The problem is optimized in "<<timer.toc()<<" seconds.###########################################"<<std::endl;

    output_file.close();


}
#endif
