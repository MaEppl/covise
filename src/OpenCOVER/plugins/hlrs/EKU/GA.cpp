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
#include<math.h>

#include "GA.hpp"

int GA::nbrCamPositions=0;
int GA::nbrPoints=0;
double GA::weightingPRIO1 = 3;
double GA::penalty =10;
int GA::populationSize = 2000;
bool GA::dynamicThreading=false;
#if(1)

std::shared_ptr<Cam> GA::getRandomCamera(int camPos, const std::function<double(void)> &rnd01)
{
    if(!camlist.at(camPos)->allCameras.empty())
    {
        int nbrCams = camlist.at(camPos)->allCameras.size();
        int r = std::roundl(rnd01()*(nbrCams-1));
        return camlist.at(camPos)->allCameras.at(r);
    }
    else
        return camlist.at(camPos)->camDraw->cam;
}

void GA::init_genes(MySolution& p,const std::function<double(void)> &rnd01)
{
    int register count =0;
    for(auto & x:p.cameras)
    {
        x=getRandomCamera(count,rnd01);
        count++;
      //  std::cout<<"random: "<<r<<st d::endl;
    }
}


bool GA::eval_solution(const MySolution& p,MyMiddleCost &c)
{   /*
     1) Überlappung der Kameras!      Position Orientation -> für 3d Rekonstruktion
     2) nur 1 Kamera gegeben und nur PRIO1 zones -> wenn visMatPrio1 genommen wird, dann keine Lösung
     3) man nimmt visMatPrio: (also Vektor wo maximal 0 oder 1 drinsteht)
        - 3 Kameras zur Verfügung, aber 2 decken bereits alle Punkte ab --> 3 ist random!


        --> ohne rejected solutions fallen Ergebnisse in jedem Durchgang sehr unterschiedlich aus!
        0 kommt raus wenn alle Punkte abgedeckt sind! was negatives wenn einige Punkte mit mehr als nötig Kameras abgedeckt sind
        positives, wenn nicht alle Punkte abgedeckt sind ->stimmt ned!!!!


        actNbrCam - minNbrCam >=0;
        0         -     1        = -2 --> multiply with factor!
        1         -     2        = -1 --> multiply with factor!

        2         -      2       =0
        3         -      2       =1 -->3 cameras are better than 2, but without factor!
    */
    std::vector<int> visYesNo_prio1(p.cameras.begin()->get()->visMatPrio1.size(),0);
    std::vector<int> visYesNo_prio2(p.cameras.begin()->get()->visMatPrio2.size(),0);

    std::vector<double> coefficients_prio1(p.cameras.begin()->get()->distortionValuePrio1.size(),1);
    std::vector<double> coefficients_prio2(p.cameras.begin()->get()->distortionValuePrio2.size(),1);

    for(const auto &x: p.cameras)
    {
      //  std::cout<<"size visMat Prio1: "<<x->visMatPrio1.size()<<std::endl;
      //  std::cout<<"size visMat Prio2: "<<x->visMatPrio2.size()<<std::endl;

        //not necessarry to add all elements, until 2 is enough
        std::transform(x->visMatPrio1.begin(),x->visMatPrio1.end(),visYesNo_prio1.begin(),visYesNo_prio1.begin(),std::plus<int>());
        std::transform(x->visMatPrio2.begin(),x->visMatPrio2.end(),visYesNo_prio2.begin(),visYesNo_prio2.begin(),std::plus<int>());

        std::transform(x->distortionValuePrio1.begin(),x->distortionValuePrio1.end(),coefficients_prio1.begin(),coefficients_prio1.begin(),std::multiplies<double>());
        std::transform(x->distortionValuePrio2.begin(),x->distortionValuePrio2.end(),coefficients_prio2.begin(),coefficients_prio2.begin(),std::multiplies<double>());

    }
    std::vector<int> percentageCalcultationPrio1;
    std::vector<int> percentageCalcultationPrio2;

    std::vector<double>visMatPrio1;
    std::vector<double>visMatPrio2;
    register int count1 =0;
    register int count2 =0;

    for(const auto&x :visYesNo_prio1)
    {
        double value = x-SafetyZone::PRIO1;
        if(value < 0)
        {
            value=value*penalty;
            visMatPrio1.push_back(value);

        }
        else
            visMatPrio1.push_back((value+1)*coefficients_prio1.at(count1));//plus 1 otherwise multiplication with zero!
        if(x>=SafetyZone::PRIO1)
            percentageCalcultationPrio1.push_back(1);
        else
           percentageCalcultationPrio1.push_back(0);

        count1++;
    }

    for(const auto&x :visYesNo_prio2)
    {
        double value = x-SafetyZone::PRIO2;
        if(value < 0)
        {
            value=value*penalty;
            visMatPrio2.push_back(value);

        }
        else
            visMatPrio2.push_back((value+1)*coefficients_prio2.at(count2));

        if(x>=SafetyZone::PRIO2)
            percentageCalcultationPrio2.push_back(1);
        else
            percentageCalcultationPrio2.push_back(0);

        count2++;
 }

    //Coverage [%]
    double sum_observed_prio1 = std::accumulate(percentageCalcultationPrio1.begin(),percentageCalcultationPrio1.end(),0.0);
    double sum_observed_prio2 = std::accumulate(percentageCalcultationPrio2.begin(),percentageCalcultationPrio2.end(),0.0);
    c.coverage.prio1 =sum_observed_prio1 / percentageCalcultationPrio1.size() *100;
    c.coverage.prio2 =sum_observed_prio2 / percentageCalcultationPrio2.size() *100;
    c.coverage.total =(sum_observed_prio2 + sum_observed_prio1) / (percentageCalcultationPrio1.size()+percentageCalcultationPrio2.size()) *100;

    static size_t counter = 0;
/*
        {   // if coverage is high enough then calc sum of PDC and RDC
            std::vector<double> visFactor_prio1(p.cameras.begin()->get()->distortionValuePrio1.size(),0);
            std::vector<double> visFactor_prio2(p.cameras.begin()->get()->distortionValuePrio2.size(),0);
            for(const auto &x: p.cameras)
            {
                std::transform(x->distortionValuePrio1.begin(),x->distortionValuePrio1.end(),visFactor_prio1.begin(),visFactor_prio1.begin(),std::plus<double>());
                std::transform(x->distortionValuePrio2.begin(),x->distortionValuePrio2.end(),visFactor_prio2.begin(),visFactor_prio2.begin(),std::plus<double>());
            }
           // c.objective = -(weighting * std::accumulate(visFactor_prio1.begin(),visFactor_prio1.end(),0.0) + std::accumulate(visFactor_prio2.begin(),visFactor_prio2.end(),0.0));
  */          c.objective = -(weightingPRIO1 * std::accumulate(visMatPrio1.begin(),visMatPrio1.end(),0.0) + std::accumulate(visMatPrio2.begin(),visMatPrio2.end(),0.0));

            return true;
/*        }
    }else
        return true;
*/
}

GA::MySolution GA:: mutate(const MySolution& X1,const std::function<double(void)> &rnd01,double shrink_scale)
{
    MySolution X_new=X1;
    int randCamPos = rnd01()*(nbrCamPositions-1);
    int count = shrink_scale * nbrCamPositions;
    while( count !=0)
    {
        X_new.cameras.at(randCamPos).reset();
        X_new.cameras.at(randCamPos) = getRandomCamera(randCamPos,rnd01);
        count--;
    }

    return X_new;
}


GA::MySolution GA::crossover(const MySolution& X1, const MySolution& X2,const std::function<double(void)> &rnd01)
{
    if(user_stop==true)
        ga_obj.user_request_stop=true;
    int cutPoint = roundl(rnd01() * nbrCamPositions);
    MySolution X_new;
    X_new.cameras.clear();
    X_new.cameras.insert(X_new.cameras.begin(),X1.cameras.begin(),X1.cameras.begin()+cutPoint);
    X_new.cameras.insert(X_new.cameras.end(),  X2.cameras.begin()+cutPoint,X2.cameras.end());

    return X_new;
}

double GA::calculate_SO_total_fitness(const GA_Type::thisChromosomeType &X)
{
    // finalize the cost
    //obtain the final cost from the middle cost
    double final_cost=0;
    final_cost+=X.middle_costs.objective;
    //ga_obj.user_request_stop=true;
    return final_cost;
}



void GA::SO_report_generation(int generation_number,const EA::GenerationType<MySolution,MyMiddleCost> &last_generation,const MySolution& best_genes)
{
    std::cout
        <<"Generation ["<<generation_number<<"], "
        <<"Best="<<last_generation.best_total_cost<<", "
        <<"Average="<<last_generation.average_cost<<", "
        <<"Best genes(cameras)=("<<best_genes.to_string()<<")"<<", "
        <<"Best Coverage[%]="<<last_generation.chromosomes.at(last_generation.best_chromosome_index).middle_costs.coverage.to_string()<<", "
        <<"Exe_time="<<last_generation.exe_time
        <<std::endl;

    finalTotalCoverage = last_generation.chromosomes.at(last_generation.best_chromosome_index).middle_costs.coverage.total;
    finalPrio1Coverage = last_generation.chromosomes.at(last_generation.best_chromosome_index).middle_costs.coverage.prio1;
    finalPrio2Coverage = last_generation.chromosomes.at(last_generation.best_chromosome_index).middle_costs.coverage.prio2;
    fitness = last_generation.best_total_cost;

    output_file
        <<generation_number<<"\t"
        <<last_generation.average_cost<<"\t"
        <<last_generation.best_total_cost<<"\t"
        <<"%.2f"<<finalTotalCoverage<<"\t"
        <<finalPrio1Coverage<<"\t"
        <<finalPrio2Coverage<<"\t"
        <<best_genes.to_string()<<"\n";

    if(user_stop==true)
    {
        std::cout<<"Was not possible to find a valid network, ABORTED (Solution not valid) !"<<std::endl;
        ga_obj.user_request_stop=true;

    }
}

std::vector<std::shared_ptr<Cam>> GA::getfinalCamPos() const
{
   std::vector<std::shared_ptr<Cam>>result= ga_obj.last_generation.chromosomes.at(ga_obj.last_generation.best_chromosome_index).genes.cameras;
   return result ;
}

GA::GA(std::vector<std::shared_ptr<CamPosition>>& cam, std::vector<std::shared_ptr<SafetyZone> > &safetyZoneList):camlist(cam)
{

    for(const auto x : safetyZoneList)
        priorityList.push_back(x->getPriority());

    int possibleCameras=0;
    for(const auto& x :camlist)
    {
        possibleCameras += x->allCameras.size();
    }
    int numberOfPoints = cam.at(0)->camDraw->cam->getVisMatPrio1().size() + cam.at(0)->camDraw->cam->getVisMatPrio2().size();
    std::string name = "results_"+std::to_string(coVRMSController::instance()->getID())+".txt";
    output_file.open(name);
    output_file<<"Nbr CamPos: "<<cam.size()<<"\t"<<"Nbr CamOrientations: "<<possibleCameras<<"\t"<<"Nbr points: "<<numberOfPoints<<"\t"<<"Population size: "<<GA::populationSize<<"\n" ;
    output_file<<"step"<<"\t"<<"cost_avg"<<"\t"<<"cost_best"<<"\t"<<"total_coverage[%]"<<"\t"<<"PRIO1_coverage[%]"<<"\t"<<"PRIO2_coverage[%]"<<"\t"<<"solution_best"<<"\n";


    std::cout<<"GA: Number of camera orientations: "<< possibleCameras<<std::endl;
    std::cout<<"GA: Number of points: "<< numberOfPoints <<std::endl;
    EA::Chronometer timer;
    timer.tic();
    using namespace std::placeholders;
    ga_obj.problem_mode=EA::GA_MODE::SOGA;
    ga_obj.multi_threading=true;
    ga_obj.idle_delay_us=1;//10 // switch between threads quickly
    ga_obj.dynamic_threading=dynamicThreading;
    ga_obj.verbose=true;
    ga_obj.population=populationSize;
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
    ga_obj.elite_count=ga_obj.population/ 100 * 6; //6% of population size;
    ga_obj.solve();

    if(!user_stop)
    {
        std::cout<<"The problem is optimized in "<<timer.toc()<<" seconds.###########################################"<<std::endl;
    }
    optimizationTime = timer.toc();
    output_file
        <<"solved in:"<<"\t"<<optimizationTime<<"\n";
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
