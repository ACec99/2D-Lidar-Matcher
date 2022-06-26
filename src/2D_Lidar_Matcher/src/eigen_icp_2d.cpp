#include "eigen_icp_2d.h"
#include "Eigen/Geometry"
#include "rotations.h"
#include "Eigen/Cholesky"
#include <iostream>
#include <fstream>
using namespace std;
using ContainerType = std::vector<Vector2f, Eigen::aligned_allocator<Vector2f> >;
using TreeNodeType = TreeNode_<typename ContainerType::iterator>;
ICP::ICP(Eigen::Isometry2f BTL_,
      Eigen::Isometry2f MTL_,
      int min_points_in_leaf,
      const int& size,
      const int& draw):
  _fixed(size),
  _moving(size),
  _min_points_in_leaf(min_points_in_leaf),
  _BTL(BTL_),
  _MTL(MTL_),_draw(draw){
  _correspondences.reserve(std::max(_fixed.size(),_moving.size()));
}

void ICP::computeCorrespondences(){
  int k=0;
  _correspondences.resize(_moving.size());
  for(const auto& m:_moving){
    const auto&  m_prime=_X*m;
    auto ft=_kd_tree->bestMatchFast(m_prime,_ball_radius);
    if(!ft)
      continue;
    else
      _correspondences[k]._fixed=*ft; //it's a list
      _correspondences[k]._moving=m_prime;
       ++k;
     }
  _correspondences.resize(k);

}

void ICP::draw(std::ostream& os) {
  os << "set size ratio -1" << endl;
  os << "plot '-' w p ps 2 title \"fixed\", '-' w p ps 2 title \"moving\", '-' w l lw 1 title \"correspondences\" " << endl;
  for  (const auto& p: _fixed)
    os <<(_MTL*p).transpose() << endl;
  os << "e" << endl;
  for  (const auto& p: _moving)
    os << (_MTL*_X*p).transpose() << endl;
  os << "e" << endl;
  for (const auto& c: _correspondences) {
    os << (_MTL*c._fixed).transpose() << endl;
    os << (_MTL*_X*c._moving).transpose() << endl;
    os << endl;
  }
  os << "e" << endl;
}
void ICP::optimizeCorrespondences() {
  Eigen::Matrix<float, 3, 3> H;
  Eigen::Matrix<float, 3, 1> b;
  
  H.setZero();
  b.setZero();
  Eigen::Matrix<float, 2, 3> J;
   J.block<2,3>(0,0).setIdentity();
  _num_kernelized=0;
  _num_inliers=0;
  _chi2_sum=0;
 
  for (const auto& c: _correspondences) {
    const auto& f=c._fixed;
    const auto& m=c._moving;
    J(0,2) = -m.y();
    J(1,2) = m.x();
    Vector2f e=m-f;
    float scale=1;
    float chi=e.squaredNorm();
    _chi2_sum+=chi;
    if (e.squaredNorm()>_kernel_chi2) {
      scale=sqrt(_kernel_chi2/chi);
      ++_num_kernelized;
    } else {
      ++_num_inliers;
    }
    H.noalias()+= scale* J.transpose()*J;
    b.noalias()+= scale* J.transpose()*e;
  }
  _dx=H.ldlt().solve(-b);
  Eigen::Isometry2f dX;
  const Eigen::Matrix2f dR=Rtheta(_dx(2));
  dX.setIdentity();
  dX.linear()=dR;
  dX.translation()=_dx.block<2,1>(0,0);
  _X=dX*_X;
  cerr << "_X matrix: " << endl;
  cerr << _X.matrix() << endl;
  
}

void ICP::run(int max_iterations) {
  _kd_tree.reset();
   _kd_tree = std::unique_ptr<TreeNodeType>(new TreeNodeType(_fixed.begin(), _fixed.end(), _min_points_in_leaf)); //initialize the kd_tree
  int current_iteration=0;
   _X=Eigen::Isometry2f::Identity();
  while (current_iteration<max_iterations) {
    computeCorrespondences();
    optimizeCorrespondences();
    if(_draw==1) draw(cout);
    ++current_iteration;
    cerr << "Iteration: " << current_iteration;
    cerr << " corr: " << numCorrespondences();
    cerr << " inl: " << numInliers();
    cerr << " ker: " << numKernelized();
    cerr << " chi: " << _chi2_sum << endl;
  
  }
}


