#include <plan_manage/planner_manager.h>
#include <thread>

namespace adaptive_planner{

AdaptivePlannerManager::AdaptivePlannerManager(){}

AdaptivePlannerManager::~AdaptivePlannerManager(){ std::cout << "des manager" << std::endl; }

void AdaptivePlannerManager::initPlanModules(ros::NodeHandle& nh){
    nh.param("fsm/safety_dist",   safety_dist_, 0.1);
    // init map
    sdf_map_.reset(new SDFMap);
    sdf_map_->initMap(nh);
    edt_environment_.reset(new EDTEnvironment);
    edt_environment_->setParam(nh);
    edt_environment_->setMap(sdf_map_);

    // init path searching
    path_finder_.reset(new Astar);
    //path_finder_.reset(new KinodynamicAstar);
    path_finder_->setParam(nh);
    //path_finder_->setEnvironment(sdf_map_);
    path_finder_->setEnvironment1(edt_environment_);
    path_finder_->init();

    low_mpc_planner_.reset(new low_mpc_planner);
    low_mpc_planner_->init(nh);
    low_mpc_planner_->setEnvironment(edt_environment_);

    high_mpcc_optimizer_.reset(new high_mpcc_optimizer);
    high_mpcc_optimizer_->setParam(nh);
    high_mpcc_optimizer_->setEnvironment(edt_environment_);

    /*
    kino_finder_.reset(new KinodynamicAstar);
    kino_finder_->setParam(nh);
    kino_finder_->setEnvironment(sdf_map_);
    kino_finder_->init();
     */
}


bool AdaptivePlannerManager::LowMpc(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel, Eigen::Vector3d end_pt){
    // global path search
    ros::Time t_start = ros::Time::now();
    int status = path_finder_->search(start_pt,end_pt);
    double t_dur = (ros::Time::now() - t_start).toSec();
    //cout << "t_dur   " << t_dur << endl;
    static int count = 0;
    if (status == 1){
        local_path_  = path_finder_->getLocalPath();
        /*
        if((local_path_[local_path_.size() -1] - start_pt).norm()< 3.0){
            count++;
        }
        cout << "kino_dynamic serach        " << (count > 1 ) << endl;
        if(count > 1){
            kino_finder_->reset();
            Eigen::Vector3d zero(0,0,0);
            int kino_status;
            kino_status = kino_finder_->search(start_pt,start_vel, zero,end_pt,zero,true);
            if (status == KinodynamicAstar::NO_PATH){
                kino_status = kino_finder_->search(start_pt,start_vel, zero,end_pt,zero,false);
                if (status == KinodynamicAstar::NO_PATH)
                    return false;
            }
            local_path_ = kino_finder_->getKinoTraj(0.04);
            count = 0;
        }
        */
        // low mpc optimization
        low_mpc_traj_  = low_mpc_planner_->lowMpcOptimization(start_pt, local_path_);
        return true;
    }else{
        return false;
    }
}

bool AdaptivePlannerManager::HighMpcc(Eigen::Matrix3d start_state, bool if_adaptive){
    // high mpcc optimization
    high_mpcc_traj_         = high_mpcc_optimizer_->mpccOptimizeTraj(start_state, low_mpc_traj_, if_adaptive);

    high_mpcc_traj_pos_     = high_mpcc_traj_[0];
    high_mpcc_traj_vel_     = high_mpcc_traj_[1];
    high_mpcc_traj_acc_     = high_mpcc_traj_[2];
    high_mpcc_traj_ref_     = high_mpcc_traj_[3];
    return true;
}

void AdaptivePlannerManager::resetMPCCinitial(){
    high_mpcc_optimizer_->resetInputInital();
}

bool AdaptivePlannerManager::safeCheck(){
    std::vector<Eigen::Vector3d> checked_tarj = low_mpc_traj_;
    int num = checked_tarj.size();
    bool if_safe = true;
    for (int i=0;i<num;i++){
        Eigen::Vector3d pos = checked_tarj[i];
        int occupied = sdf_map_->getInflateOccupancy(pos);
        if (occupied == 1){
            if_safe = false;
            break;
        }    
    }
    return if_safe;
}

}	// namespace adaptive_planner