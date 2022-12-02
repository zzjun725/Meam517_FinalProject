#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Dense>
#include "OsqpEigen/OsqpEigen.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <vector>

using namespace std;
using namespace Eigen;

// subscriber
ros::Subscriber corridorSub, targetSub;

// publisher
ros::Publisher trajPub;

// parameters
int numSegments;
int order = 8;
vector<double> scales;
visualization_msgs::MarkerArray savedCorridors;

// p, v, a constraint
vector<vector<double>> startConstraint(3, vector<double>(3, 0.0));
vector<vector<double>> targetConstraint(3, vector<double>(3, 0.0));

// boundary
vector<vector<double>> lowerBoundary(3);
vector<vector<double>> upperBoundary(3);

// val, acc limit
double maxVal = 5.0;
double maxAcc = 5.0;

// output
vector<VectorXd> trajControlPoints(3);

// receive signal
bool corridorRcv = false;
bool targetRcv = false;

// get M matrix to transform between Bezier coefficients and polynomial coefficients, with c.T @ M.T = p.T
MatrixXd getM(int order) {
    MatrixXd M;
    M.resize(order + 1, order + 1);
    switch(order)
    {	
        case 0: 
        {
            M << 1;
            break;
        }
        case 1: 
        {
            M << -1,  0,
                -1,  1;
            break;
        }
        case 2:
        {
            M << -1,  0,  0,
                -2,  2,  0,
                1, -2,  1;
            break;
        }
        case 3: 
        {
            M << -1,  0,  0,  0,
                -3,  3,  0,  0,
                3, -6,  3,  0,
                -1,  3, -3,  1;	
            break;
        }
        case 4:
        {
            M << 1,   0,   0,   0,  0,
                -4,   4,   0,   0,  0,
                6, -12,   6,   0,  0,
                -4,  12, -12,   4,  0,
                1,  -4,   6,  -4,  1;
            break;
        }
        case 5:
        {
            M << 1,   0,   0,   0,  0,  0,
                -5,   5,   0,   0,  0,  0,
                10, -20,  10,   0,  0,  0,
                -10,  30, -30,  10,  0,  0,
                5, -20,  30, -20,  5,  0,
                -1,   5, -10,  10, -5,  1;
            break;
        }
        case 6:
        {	
            M << 1,   0,   0,   0,   0,  0,  0,
                -6,   6,   0,   0,   0,  0,  0,
                15, -30,  15,   0,   0,  0,  0,
                -20,  60, -60,  20,   0,  0,  0,
                15, -60,  90, -60,  15,  0,  0,
                -6,  30, -60,  60, -30,  6,  0,
                1,  -6,  15, -20,  15, -6,  1;
            break;
        }
        case 7:
        {
            M << 1,    0,    0,    0,    0,   0,   0,   0,
                -7,    7,    0,    0,    0,   0,   0,   0,
                21,   42,   21,    0,    0,   0,   0,   0,
                -35,  105, -105,   35,    0,   0,   0,   0, 
                35, -140,  210, -140,   35,   0,   0,   0,
                -21,  105, -210,  210, -105,  21,   0,   0,
                    7,  -42,  105, -140,  105, -42,   7,   0,
                -1,    7,  -21,   35,  -35,  21,  -7,   1;
            break;
        }
        case 8:
        {
            M << 1,    0,    0,    0,    0,    0,   0,   0,   0,
                -8,    8,    0,    0,    0,    0,   0,   0,   0,
                28,  -56,   28,    0,    0,    0,   0,   0,   0,
                -56,  168, -168,   56,    0,    0,   0,   0,   0, 
                70, -280,  420, -280,   70,    0,   0,   0,   0,
                -56,  280, -560,  560, -280,   56,   0,   0,   0,
                28, -168,  420, -560,  420, -168,  28,   0,   0,
                -8,   56, -168,  280, -280,  168, -56,   8,   0,
                1,   -8,   28,  -56,   70,  -56,  28,  -8,   1;
            break;
        }
        case 9:
        {
            M << 1,    0,     0,     0,     0,    0,    0,     0,     0,    0,
                -9,    9,     0,     0,     0,    0,    0,     0,     0,    0, 
                36,  -72,    36,     0,     0,    0,    0,     0,     0,    0, 
                -84,  252,  -252,    84,     0,    0,    0,     0,     0,    0, 
                126, -504,   756,  -504,   126,    0,    0,     0,     0,    0,
                -126,  630, -1260,  1260,  -630,  126,    0,     0,     0,    0,
                84, -504,  1260, -1680,  1260, -504,   84,     0,     0,    0,
                -36,  252,  -756,  1260, -1260,  756, -252,    36,     0,    0,
                9,  -72,   252,  -504,   630, -504,  252,   -72,     9,    0,
                -1,    9,   -36,    84,  -126,  126,  -84,    36,    -9,    1;
            break;
        }
        case 10:
        {
            M <<  1,     0,     0,     0,      0,     0,    0,     0,     0,    0,   0,
                -10,    10,     0,     0,      0,     0,    0,     0,     0,    0,   0,
                45,   -90,    45,     0,      0,     0,    0,     0,     0,    0,   0,
                -120,   360,  -360,   120,      0,     0,    0,     0,     0,    0,   0,
                210,  -840,  1260,  -840,    210,     0,    0,     0,     0,    0,   0,
                -252,  1260, -2520,  2520,  -1260,   252,    0,     0,     0,    0,   0,
                210, -1260,  3150, -4200,   3150, -1260,  210,     0,     0,    0,   0,
                -120,  840,  -2520,  4200,  -4200,  2520, -840,   120,     0,    0,   0,
                45, -360,   1260, -2520,   3150, -2520, 1260,  -360,    45,    0,   0,
                -10,   90,   -360,   840,  -1260,  1260, -840,   360,   -90,   10,   0,
                1,  -10,     45,  -120,    210,  -252,  210,  -120,    45,  -10,   1;
            break;
        }
        case 11:
        {
            M <<  1,     0,    0,      0,      0,      0,     0,     0,     0,    0,   0,  0,
                -11,    11,    0,      0,      0,      0,     0,     0,     0,    0,   0,  0,
                55,  -110,   55,      0,      0,      0,     0,     0,     0,    0,   0,  0,
                -165,   495, -495,    165,      0,      0,     0,     0,     0,    0,   0,  0,
                330, -1320, 1980,  -1320,    330,      0,     0,     0,     0,    0,   0,  0,
                -462,  2310, -4620,  4620,  -2310,    462,     0,     0,     0,    0,   0,  0,
                462, -2772,  6930, -9240,   6930,  -2772,   462,     0,     0,    0,   0,  0,
                -330,  2310, -6930, 11550, -11550,   6930, -2310,   330,     0,    0,   0,  0,
                165, -1320,  4620, -9240,  11550,  -9240,  4620, -1320,   165,    0,   0,  0,
                -55,   495, -1980,  4620,  -6930,   6930, -4620,  1980,  -495,   55,   0,  0,
                11,  -110,   495, -1320,   2310,  -2772,  2310, -1320,   495, -110,  11,  0,
                -1,    11,   -55,   165,   -330,    462,  -462,   330,  -165,   55, -11,  1;
            break;
        }
        case 12:
        {
            M <<  1,     0,     0,      0,      0,      0,     0,     0,     0,    0,    0,   0,   0,
                -12,    12,     0,      0,      0,      0,     0,     0,     0,    0,    0,   0,   0,
                66,  -132,    66,      0,      0,      0,     0,     0,     0,    0,    0,   0,   0,
                -220,   660,  -660,    220,      0,      0,     0,     0,     0,    0,    0,   0,   0,
                495, -1980,  2970,  -1980,    495,      0,     0,     0,     0,    0,    0,   0,   0, 
                -792,  3960, -7920,   7920,  -3960,    792,     0,     0,     0,    0,    0,   0,   0,
                924, -5544, 13860, -18480,  13860,  -5544,   924,     0,     0,    0,    0,   0,   0,
                -792,  5544,-16632,  27720, -27720,  16632, -5544,   792,     0,    0,    0,   0,   0,
                495, -3960, 13860, -27720,  34650, -27720, 13860, -3960,   495,    0,    0,   0,   0,
                -220,  1980, -7920,  18480, -27720,  27720,-18480,  7920, -1980,  220,    0,   0,   0,
                66,  -660,  2970,  -7920,  13860, -16632, 13860, -7920,  2970, -660,   66,   0,   0,
                -12,   132,  -660,   1980,  -3960,   5544, -5544,  3960, -1980,  660, -132,  12,   0,
                1,   -12,    66,   -220,    495,   -792,   924,  -792,   495, -220,   66, -12,   1;
            break;
        }
    }

    return M;
}

// get Q matrix for the objective function
MatrixXd getQ(int order){
    MatrixXd Q = MatrixXd::Zero(order + 1, order + 1);

    for (int i = 3; i < order + 1; i++) {
        for (int j = 3; j < order + 1; j++) {
            Q(i, j) = i * (i - 1) * (i - 2) * j * (j - 1) * (j - 2) / (i + j - 5);
        }
    }

    return Q;
}

// get waypoint and continuity constraints
int getWaypointAndContinuityConstraints(int dim, int numSegments, int order, int startRow, SparseMatrix<double>& linearMatrix, VectorXd& lowerBound, VectorXd& upperBound) {
    int rowIdx = startRow;
    // start position
    // position
    linearMatrix.insert(rowIdx, 0) = scales[0];
    lowerBound(rowIdx) = startConstraint[dim][0];
    upperBound(rowIdx) = startConstraint[dim][0];
    rowIdx++;

    // velocity
    linearMatrix.insert(rowIdx, 0) = -order;
    linearMatrix.insert(rowIdx, 1) = order;
    lowerBound(rowIdx) = startConstraint[dim][1];
    upperBound(rowIdx) = startConstraint[dim][1];
    rowIdx++;

    // acceleration
    linearMatrix.insert(rowIdx, 0) = order * (order - 1) / scales[0];
    linearMatrix.insert(rowIdx, 1) = -2 * order * (order - 1) / scales[0];
    linearMatrix.insert(rowIdx, 2) = order * (order - 1) / scales[0];
    lowerBound(rowIdx) = startConstraint[dim][2];
    upperBound(rowIdx) = startConstraint[dim][2];
    rowIdx++;

    // end position
    // position
    linearMatrix.insert(rowIdx, numSegments * (order + 1) - 1) = scales.back();
    lowerBound(rowIdx) = targetConstraint[dim][0];
    upperBound(rowIdx) = targetConstraint[dim][0];
    rowIdx++;

    // velocity
    linearMatrix.insert(rowIdx, numSegments * (order + 1) - 2) = -order;
    linearMatrix.insert(rowIdx, numSegments * (order + 1) - 1) = order;
    lowerBound(rowIdx) = targetConstraint[dim][1];
    upperBound(rowIdx) = targetConstraint[dim][1];
    rowIdx++;

    // acceleration
    linearMatrix.insert(rowIdx, numSegments * (order + 1) - 3) = order * (order - 1) / scales.back();
    linearMatrix.insert(rowIdx, numSegments * (order + 1) - 2) = -2 * order * (order - 1) / scales.back();
    linearMatrix.insert(rowIdx, numSegments * (order + 1) - 1) = order * (order - 1) / scales.back();
    lowerBound(rowIdx) = targetConstraint[dim][2];
    upperBound(rowIdx) = targetConstraint[dim][2];
    rowIdx++;
    
    // mid positions
    for (int i = 0; i < numSegments - 1; i++) {
        // position
        linearMatrix.insert(rowIdx, (i + 1) * (order + 1) - 1) = scales[i];
        linearMatrix.insert(rowIdx, (i + 1) * (order + 1)) = -scales[i + 1];
        lowerBound(rowIdx) = 0.0;
        upperBound(rowIdx) = 0.0;
        rowIdx++;

        // velocity
        linearMatrix.insert(rowIdx, (i + 1) * (order + 1) - 2) = -order;
        linearMatrix.insert(rowIdx, (i + 1) * (order + 1) - 1) = order;
        linearMatrix.insert(rowIdx, (i + 1) * (order + 1)) = order;
        linearMatrix.insert(rowIdx, (i + 1) * (order + 1) + 1) = -order;
        lowerBound(rowIdx) = 0.0;
        upperBound(rowIdx) = 0.0;
        rowIdx++;

        // acceleration
        linearMatrix.insert(rowIdx, (i + 1) * (order + 1) - 3) = order * (order - 1) / scales[i];
        linearMatrix.insert(rowIdx, (i + 1) * (order + 1) - 2) = -2 * order * (order - 1) / scales[i];
        linearMatrix.insert(rowIdx, (i + 1) * (order + 1) - 1) = order * (order - 1) / scales[i];
        linearMatrix.insert(rowIdx, (i + 1) * (order + 1)) = -order * (order - 1) / scales[i + 1];
        linearMatrix.insert(rowIdx, (i + 1) * (order + 1) + 1) = 2 * order * (order - 1) / scales[i + 1];
        linearMatrix.insert(rowIdx, (i + 1) * (order + 1) + 2) = -order * (order - 1) / scales[i + 1];
        lowerBound(rowIdx) = 0.0;
        upperBound(rowIdx) = 0.0;
        rowIdx++;
    }

    return rowIdx;
}

// get safety constraints
int getSafetyConstraints(int dim, int numSegments, int order, int startRow, SparseMatrix<double>& linearMatrix, VectorXd& lowerBound, VectorXd& upperBound) {
    int rowIdx = startRow;

    for (int i = 0; i < numSegments; i++) {
        for (int j = 0; j < order + 1; j++) {
            linearMatrix.insert(rowIdx, i * (order + 1) + j) = 1.0;
            lowerBound(rowIdx) = lowerBoundary[dim][i] / scales[i];
            upperBound(rowIdx) = upperBoundary[dim][i] / scales[i];
            rowIdx++;
        }
    }

    return rowIdx;
}

// get dynamical feasibility constraints
int getDynamicalFeasibilityConstraints(int numSegments, int order, int startRow, SparseMatrix<double>& linearMatrix, VectorXd& lowerBound, VectorXd& upperBound) {
    int rowIdx = startRow;

    for (int i = 0; i < numSegments; i++) {
        for (int j = 0; j < order; j++) {
            linearMatrix.insert(rowIdx, i * (order + 1) + j) = -order;
            linearMatrix.insert(rowIdx, i * (order + 1) + j + 1) = order;
            lowerBound(rowIdx) = -maxVal;
            upperBound(rowIdx) = maxVal;
            rowIdx++;
        }
    }

    for (int i = 0; i < numSegments; i++) {
        for (int j = 0; j < order - 1; j++) {
            linearMatrix.insert(rowIdx, i * (order + 1) + j) = order * (order - 1) / scales[i];
            linearMatrix.insert(rowIdx, i * (order + 1) + j + 1) = -2 * order * (order - 1) / scales[i];
            linearMatrix.insert(rowIdx, i * (order + 1) + j + 2) = order * (order - 1) / scales[i];
            lowerBound(rowIdx) = -maxAcc;
            upperBound(rowIdx) = maxAcc;
            rowIdx++;
        }
    }

    return rowIdx;
}

void calculateScales() {
    scales.clear();
    scales.resize(numSegments);

    vector<Vector3d> points;
    points.emplace_back(startConstraint[0][0], startConstraint[1][0], startConstraint[2][0]);

    for (int i = 1; i < (int)savedCorridors.markers.size(); i++) {
        const auto& box = savedCorridors.markers[i];
        points.emplace_back(box.pose.position.x, box.pose.position.y, box.pose.position.z);
    }

    points.emplace_back(targetConstraint[0][0], targetConstraint[1][0], targetConstraint[2][0]);

    double _Vel = maxVal * 0.6;
    double _Acc = maxAcc * 0.6;

    for (int k = 0; k < (int)points.size() - 1; k++)
    {   
        double dtxyz;
        Vector3d p0   = points[k];        
        Vector3d p1   = points[k + 1];    
        Vector3d d    = p1 - p0;          
        Vector3d v0(0.0, 0.0, 0.0);       
        
        // if( k == 0) v0 = _start_vel;

        double D    = d.norm();                  
        double V0   = v0.dot(d / D);             
        double aV0  = fabs(V0);                  

        double acct = (_Vel - V0) / _Acc * ((_Vel > V0)? 1 : -1);
        double accd = V0 * acct + (_Acc * acct * acct / 2) * ((_Vel > V0)? 1 : -1);
        double dcct = _Vel / _Acc;                                              
        double dccd = _Acc * dcct * dcct / 2;                                   

        if (D < aV0 * aV0 / (2 * _Acc))
        {               
            double t1 = (V0 < 0)?2.0 * aV0 / _Acc:0.0;
            double t2 = aV0 / _Acc;
            dtxyz     = t1 + t2;                 
        }
        else if (D < accd + dccd)
        {
            double t1 = (V0 < 0)?2.0 * aV0 / _Acc:0.0;
            double t2 = (-aV0 + sqrt(aV0 * aV0 + _Acc * D - aV0 * aV0 / 2)) / _Acc;
            double t3 = (aV0 + _Acc * t2) / _Acc;
            dtxyz     = t1 + t2 + t3;    
        }
        else
        {
            double t1 = acct;                              
            double t2 = (D - accd - dccd) / _Vel;
            double t3 = dcct;
            dtxyz     = t1 + t2 + t3;                                                                  
        }
        scales[k] = dtxyz;
    }
}

int factorial(int n) {
    return (n == 1 || n == 0) ? 1 : factorial(n - 1) * n;
}

void visualize() {
    visualization_msgs::Marker line_strip;
    line_strip.header.frame_id = "map";
    line_strip.header.stamp = ros::Time::now();
    line_strip.ns = "points_and_lines";
    line_strip.action = visualization_msgs::Marker::ADD;
    line_strip.pose.orientation.w = 1.0;

    line_strip.id = 1;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    line_strip.scale.x = 0.1;
    // Line strip is blue
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;

    // Create the vertices for the lines
    double total_time = 0.0;
    for (int i = 0; i < numSegments; i++) {
        vector<double> ts(1000, 0);
        for (int j = 0; j < 1000; j++) {
            ts[j] = total_time + scales[i] / 1000 * j;
        }
        vector<vector<double>> fs(3, vector<double>(1000, 0));
        for (int d = 0; d < 3; d++) {
            for (int o = 0; o < order + 1; o++) {
                for (int j = 0; j < 1000; j++) {
                    fs[d][j] += scales[i] * trajControlPoints[d](i * (order + 1) + o) * factorial(order) / factorial(o) / factorial(order - o) * pow((ts[j] - total_time) / scales[i], o) * pow(1 - (ts[j] - total_time) / scales[i] , order - o);
                }
            }

        }

        total_time += scales[i];
        
        for (int j = 0; j < 1000; j++) {
            geometry_msgs::Point p;
            p.x = fs[0][j];
            p.y = fs[1][j];
            p.z = fs[2][j];
            line_strip.points.push_back(p);
        }
    }

    trajPub.publish(line_strip);
}

int solve() {
    // calculate scales
    calculateScales();

    // cout << "goal" << targetConstraint[0][0] << ' ' << targetConstraint[1][0] << ' ' << targetConstraint[2][0] << endl;

    // for (double ele : scales) {
    //     cout << ele << ' ';
    // }
    // cout << endl;

    // for (int i = 0; i < numSegments; i++) {
    //     cout << "lb:" << endl;
    //     cout << lowerBoundary[0][i] << ", " << lowerBoundary[1][i] << ", " << lowerBoundary[2][i] << endl;
    //     cout << "ub:" << endl;
    //     cout << upperBoundary[0][i] << ", " << upperBoundary[1][i] << ", " << upperBoundary[2][i] << endl;
    // }

    // cost part
    MatrixXd Q = getQ(order);
    MatrixXd M = getM(order);
    MatrixXd Q0 = M.transpose() * Q * M;

    // cout << "Q:" << Q << endl;
    
    // find closest PSD Qhat
    MatrixXd B = (Q0 + Q0.transpose()) / 2;
    JacobiSVD<MatrixXd> svd(B, ComputeFullV | ComputeFullU);
    MatrixXd H = svd.matrixV() * svd.singularValues().asDiagonal() * svd.matrixV().transpose();
    MatrixXd Qhat = (B + H) / 2;
    
    MatrixXd blockQ0 = MatrixXd::Zero(Q0.rows() * numSegments, Q0.cols() * numSegments);
    for (int i = 0; i < numSegments; i++)
    {
        blockQ0.block(i * Q0.rows(), i * Q0.cols(), Q0.rows(), Q0.cols()) = Qhat / pow(scales[i], 3);
    }
    SparseMatrix<double> sparseBlockQ0(blockQ0.sparseView());

    VectorXd gradient = VectorXd::Zero(numSegments * (order + 1));

    // solve x, y, z separately
    for (int dim = 0; dim < 3; dim++) {
        // constraint part
        SparseMatrix<double> linearMatrix;
        VectorXd lowerBound, upperBound;
        int totalConstraintNum = 3 * numSegments + 3 + 3 * numSegments * order;
        linearMatrix.resize(totalConstraintNum, numSegments * (order + 1));
        lowerBound.resize(totalConstraintNum);
        upperBound.resize(totalConstraintNum);

        int startRow = getWaypointAndContinuityConstraints(dim, numSegments, order, 0, linearMatrix, lowerBound, upperBound);

        startRow = getSafetyConstraints(dim, numSegments, order, startRow, linearMatrix, lowerBound, upperBound);

        startRow = getDynamicalFeasibilityConstraints(numSegments, order, startRow, linearMatrix, lowerBound, upperBound);

        // solve x, y, z separately with 3 optimization problems
        // instantiate the solver
        OsqpEigen::Solver solver;

        // settings
        //solver.settings()->setVerbosity(false);
        solver.settings()->setWarmStart(true);
        
        // set the initial data of the QP solver
        solver.data()->setNumberOfVariables(numSegments * (order + 1));
        solver.data()->setNumberOfConstraints(totalConstraintNum);
        if(!solver.data()->setHessianMatrix(sparseBlockQ0)) return 1;
        if(!solver.data()->setGradient(gradient)) return 1;
        if(!solver.data()->setLinearConstraintsMatrix(linearMatrix)) return 1;
        if(!solver.data()->setLowerBound(lowerBound)) return 1;
        if(!solver.data()->setUpperBound(upperBound)) return 1;

        // instantiate the solver
        if(!solver.initSolver()) return 1;

        // controller input and QPSolution vector
        VectorXd controlPoints;

        // solve the QP problem
        if(solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return 1;

        if(solver.getStatus() != OsqpEigen::Status::Solved) return 1;

        // get the controller input
        controlPoints = solver.getSolution();

        trajControlPoints[dim] = controlPoints;

        // cout << "dim: " << dim << endl;
        // cout << controlPoints << endl;
        // cout << endl;
    }

    ofstream myfile;
    myfile.open("/home/wqr/control_point.txt");
    for (const auto& cp : trajControlPoints) {
        myfile << cp;
        myfile << endl;
    }
    myfile.close();

    visualize();

    return 0;
}

void rcvCorridorsCallBack(const visualization_msgs::MarkerArray& corridors) {
    if (corridors.markers.size() == 0)
        return;

    savedCorridors = corridors;

    numSegments = corridors.markers.size();

    cout << "numSegments: " << numSegments << endl;
    
    // set boundary
    for (int i = 0; i < 3; i++) {
        lowerBoundary[i].clear();
        upperBoundary[i].clear();
        lowerBoundary[i].resize(numSegments);
        upperBoundary[i].resize(numSegments);
    }

    for (int i = 0; i < numSegments; i++) {
        const auto& box = corridors.markers[i];

        lowerBoundary[0][i] = box.pose.position.x - box.scale.x / 2.0;
        lowerBoundary[1][i] = box.pose.position.y - box.scale.y / 2.0;
        lowerBoundary[2][i] = box.pose.position.z - box.scale.z / 2.0;
        
        upperBoundary[0][i] = box.pose.position.x + box.scale.x / 2.0;
        upperBoundary[1][i] = box.pose.position.y + box.scale.y / 2.0;
        upperBoundary[2][i] = box.pose.position.z + box.scale.z / 2.0;
    }

    corridorRcv = true;

    if (corridorRcv && targetRcv) {
        int error = solve();
        corridorRcv = false;
        targetRcv = false;
        if (error) {
            ROS_WARN("Btraj failed!");
        }
    }
}

void rcvTargetCallback(const geometry_msgs::PoseStamped& target) {     
    targetConstraint[0][0] = target.pose.position.x;
    targetConstraint[1][0] = target.pose.position.y;
    targetConstraint[2][0] = target.pose.position.z;
    targetRcv = true;

    if (corridorRcv && targetRcv) {
        int error = solve();
        corridorRcv = false;
        targetRcv = false;
        if (error) {
            ROS_WARN("Btraj failed!");
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "btraj_main");
    ros::NodeHandle nh("~");

    // subscriber
    corridorSub = nh.subscribe("/cubic_corridors", 1, rcvCorridorsCallBack);
    targetSub = nh.subscribe("/goal", 1, rcvTargetCallback);

    // publisher
    trajPub = nh.advertise<visualization_msgs::Marker>("curve", 10);
    
    sleep(1);
    ros::spin();
    
    return 0;
}