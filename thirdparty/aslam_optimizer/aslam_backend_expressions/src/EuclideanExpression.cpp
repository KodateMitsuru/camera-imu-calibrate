#include <aslam/backend/EuclideanExpression.hpp>
#include <aslam/backend/EuclideanExpressionNode.hpp>
#include <aslam/backend/HomogeneousExpression.hpp>
#include <aslam/backend/HomogeneousExpressionNode.hpp>
#include <aslam/backend/ScalarExpression.hpp>
#include <aslam/backend/ScalarExpressionNode.hpp>
#include <aslam/backend/VectorExpression.hpp>
#include <aslam/backend/VectorExpressionNode.hpp>
#include <sm/boost/null_deleter.hpp>

namespace aslam {
namespace backend {
EuclideanExpression::EuclideanExpression() {}

EuclideanExpression::EuclideanExpression(
    EuclideanExpressionNode* designVariable)
    : _root(designVariable, sm::null_deleter()) {}

EuclideanExpression::EuclideanExpression(
    std::shared_ptr<EuclideanExpressionNode> root)
    : _root(root) {}

EuclideanExpression::EuclideanExpression(
    const VectorExpression<3>& vectorExpression)
    : _root(new VectorExpression2EuclideanExpressionAdapter(
          vectorExpression.root())) {}

EuclideanExpression::EuclideanExpression(const Eigen::Vector3d& p)
    : _root(new EuclideanExpressionNodeConstant(p)) {}

EuclideanExpression::~EuclideanExpression() {}

Eigen::Vector3d EuclideanExpression::toEuclidean() const {
  return _root->toEuclidean();
}

HomogeneousExpression EuclideanExpression::toHomogeneousExpression() const {
  std::shared_ptr<HomogeneousExpressionNode> newRoot(
      new HomogeneousExpressionNodeEuclidean(_root));
  return HomogeneousExpression(newRoot);
}

void EuclideanExpression::evaluateJacobians(
    JacobianContainer& outJacobians) const {
  _root->evaluateJacobians(outJacobians);
}

void EuclideanExpression::evaluateJacobians(
    JacobianContainer& outJacobians,
    const Eigen::MatrixXd& applyChainRule) const {
  _root->evaluateJacobians(outJacobians, applyChainRule);
}

EuclideanExpression EuclideanExpression::cross(
    const EuclideanExpression& p) const {
  std::shared_ptr<EuclideanExpressionNode> newRoot(
      new EuclideanExpressionNodeCrossEuclidean(_root, p._root));
  return EuclideanExpression(newRoot);
}

EuclideanExpression EuclideanExpression::elementwiseMultiply(
    const EuclideanExpression& p) const {
  std::shared_ptr<EuclideanExpressionNode> newRoot(
      new EuclideanExpressionNodeElementwiseMultiplyEuclidean(_root, p._root));
  return EuclideanExpression(newRoot);
}

EuclideanExpression EuclideanExpression::operator+(
    const EuclideanExpression& p) const {
  std::shared_ptr<EuclideanExpressionNode> newRoot(
      new EuclideanExpressionNodeAddEuclidean(_root, p._root));
  return EuclideanExpression(newRoot);
}

EuclideanExpression EuclideanExpression::operator-(
    const EuclideanExpression& p) const {
  std::shared_ptr<EuclideanExpressionNode> newRoot(
      new EuclideanExpressionNodeSubtractEuclidean(_root, p._root));
  return EuclideanExpression(newRoot);
}

EuclideanExpression EuclideanExpression::operator-(
    const Eigen::Vector3d& p) const {
  std::shared_ptr<EuclideanExpressionNode> newRoot(
      new EuclideanExpressionNodeSubtractVector(_root, p));
  return EuclideanExpression(newRoot);
}

EuclideanExpression EuclideanExpression::operator-() const {
  std::shared_ptr<EuclideanExpressionNode> newRoot(
      new EuclideanExpressionNodeNegated(_root));
  return EuclideanExpression(newRoot);
}

EuclideanExpression EuclideanExpression::operator*(
    const ScalarExpression& s) const {
  std::shared_ptr<EuclideanExpressionNode> newRoot(
      new EuclideanExpressionNodeScalarMultiply(_root, s._root));
  return EuclideanExpression(newRoot);
}

void EuclideanExpression::getDesignVariables(
    DesignVariable::set_t& designVariables) const {
  _root->getDesignVariables(designVariables);
}

}  // namespace backend
}  // namespace aslam
