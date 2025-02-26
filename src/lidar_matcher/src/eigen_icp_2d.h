#pragma once
#include "eigen_kdtree.h"
#include "Eigen/Geometry"
#include <iostream>
#include <list>
#include <memory>
#include <iostream>
#include <fstream>
using namespace std;
using Vector2f = Eigen::Vector2f;

class ICP {
protected:
  struct PointPair{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    PointPair(const Vector2f& fixed_, const Vector2f& moving_):
      _fixed(fixed_),
      _moving(moving_){};
    
    PointPair(){}
    Vector2f _fixed; //the point in the fixed cloud
    Vector2f _moving; //the point in the moving cloud
  };
  using PointPairVector=std::vector<PointPair, Eigen::aligned_allocator<PointPair>>; //this is the declaration of a class/object that is a vector of 	PointPairs => vector of correspondeces 

public:
  using ContainerType=std::vector<Vector2f, Eigen::aligned_allocator<Vector2f> >; 
  
  ICP(Eigen::Isometry2f BTL_,
      Eigen::Isometry2f MTL_,
      int min_points_in_leaf,
      const int& size);
      
      void computeCorrespondences();
      
      void computeCorrespondencesFake();
  
      void optimizeCorrespondences();
  
  
      void run(int max_iterations);
      
      void draw(std::ostream& os);
      
      const Eigen::Isometry2f& X() const {return _X;}
      Eigen::Isometry2f& X()  {return _X;}
  
      // The pose of the base_link frame wrt map frame.
      Eigen::Isometry2f MTB() const {return _MTL*(_BTL.inverse());}
      void updateMTL() {_MTL=_MTL*X();}; //update the isometry
      
      void updateOld(){
      _fixed.swap(_moving);
     
       } //change the old one with the new one
      
       /*inline int ValuesInsertionTest(const int idx, Eigen::Vector2f value){
       		if ( _fixed[idx] == value  ) return 1;
		
		else return 0;
	}*/
       void ValuesInsertion(bool Moving_or_Fixed, const int i, Eigen::Vector2f value){
       		if(Moving_or_Fixed) _moving[i]=value;
       		else _fixed[i]=value;
  	}
  
  inline int numCorrespondences() const {return _correspondences.size();}
  inline int numKernelized() const {return _num_kernelized;}
  inline int numInliers() const {return _num_inliers;}
  inline const Eigen::Matrix<float, 3,1>& dx() const {return _dx;}

  
  //fillme, it is gonna be easy
protected:
  using TreeNodeType = TreeNode_<typename ContainerType::iterator>;
  ContainerType _fixed;
  ContainerType _moving;
  Eigen::Isometry2f _X=Eigen::Isometry2f::Identity();//Eigen::Isometry2f::Identity();
 
  Eigen::Isometry2f _BTL; //The pose of the laser_frame wrt base_link

  Eigen::Isometry2f _MTL;  //The pose of the laser_frame wrt map.
  std::unique_ptr<TreeNodeType>  _kd_tree;
  int _min_points_in_leaf;
  //int _draw;
  float _ball_radius=10.f;
  float _kernel_chi2 = 1.f;
  float _chi2_sum=0;

  PointPairVector _correspondences;
  int _num_kernelized=0;
  int _num_inliers=0;
  Eigen::Matrix<float, 3,1> _dx;
};
