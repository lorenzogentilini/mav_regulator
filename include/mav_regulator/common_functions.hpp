#pragma once

template <typename T>
void fillRawBuffer(real_t *rawBuffer, T matrix, uint replicate){
    // ACADO Memorizes Vectors in ROW MAJOR Format
    for(uint rr = 0; rr < replicate; rr++){
        for(uint ii = 0; ii < matrix.rows(); ii++){
            for(uint jj = 0; jj < matrix.cols(); jj++){
                rawBuffer[matrix.size()*rr + matrix.cols()*ii + jj] = matrix(ii, jj);
            }
        }
    }
}

double deBoor(int n, int k, double time, std::vector<float> knot){
    double c1, c2;
    if(n == 0){
        if((time >= knot[k]) && (time < knot[k+1]))
            return 1;
        else
            return 0;
    } else{
        if(knot[k+n] - knot[k] != 0)
            c1 = (time - knot[k])/(knot[k+n] - knot[k]);
        else
            c1 = 0;

        if(knot[k+n+1] - knot[k+1] != 0)
            c2 = (knot[k+n+1] - time)/(knot[k+n+1] - knot[k+1]);
        else
            c2 = 0;
        
        return c1*deBoor(n-1, k, time, knot) + c2*deBoor(n-1, k+1, time, knot);
    }
}

inline int fac(int n){
    if(n > 1){
        return n*fac(n-1);
    } else{
        return 1;
    }
}

inline float bin(int n, int k){
    if(k <= n){
        return ((float)fac(n)/((float)fac(k)*(float)fac(n - k)));
    } else{
        return 0;
    }
}

template <typename T>
inline T max(T a, T b){
    if(a > b){
        return a;
    } else{
        return b;
    }
}

template <typename T>
inline T min(T a, T b){
    if(a < b){
        return a;
    } else{
        return b;
    }
}

template <typename T>
T evaluateBezier_general(std::vector<T> controlPoints, float u){
    switch(controlPoints.size()-1){
        case 2:{
            return pow(1-u, 2)*controlPoints[0] + 2*u*(1-u)*controlPoints[1] + pow(u, 2)*controlPoints[2];
            break;
        }

        case 3:{
            return pow(1-u, 3)*controlPoints[0] + 3*u*pow(1-u, 2)*controlPoints[1] + 3*pow(u, 2)*(1-u)*controlPoints[2] + pow(u, 3)*controlPoints[3];
            break;
        }

        case 4:{
            return pow(1-u, 4)*controlPoints[0] + 4*u*pow(1-u, 3)*controlPoints[1] + 6*pow(u, 2)*pow(1-u, 2)*controlPoints[2] + 4*pow(u, 3)*(1-u)*controlPoints[3] + pow(u, 4)*controlPoints[4];
            break;
        }

        case 5:{
            return pow(1-u, 5)*controlPoints[0] + 5*u*pow(1-u, 4)*controlPoints[1] + 10*pow(u, 2)*pow(1-u, 3)*controlPoints[2] + 10*pow(u, 3)*pow(1-u, 2)*controlPoints[3] + 5*pow(u, 4)*(1-u)*controlPoints[4] + pow(u, 5)*controlPoints[5];
            break;
        }

        case 6:{
            return pow(1-u, 6)*controlPoints[0] + 6*u*pow(1-u, 5)*controlPoints[1] + 15*pow(u, 2)*pow(1-u, 4)*controlPoints[2] + 20*pow(u, 3)*pow(1-u, 3)*controlPoints[3] + 15*pow(u, 4)*pow(1-u, 2)*controlPoints[4] + 6*pow(u, 5)*(1-u)*controlPoints[5] + pow(u, 6)*controlPoints[6];
            break;
        }

        case 7:{
            return pow(1-u, 7)*controlPoints[0] + 7*u*pow(1-u, 6)*controlPoints[1] + 21*pow(u, 2)*pow(1-u, 5)*controlPoints[2] + 35*pow(u, 3)*pow(1-u, 4)*controlPoints[3] + 35*pow(u, 4)*pow(1-u, 3)*controlPoints[4] + 21*pow(u, 5)*pow(1-u, 2)*controlPoints[5] + 7*pow(u, 6)*(1-u)*controlPoints[6] + pow(u, 7)*controlPoints[7];
            break;
        }

        case 8:{
            return pow(1-u, 8)*controlPoints[0] + 8*u*pow(1-u, 7)*controlPoints[1] + 28*pow(u, 2)*pow(1-u, 6)*controlPoints[2] + 56*pow(u, 3)*pow(1-u, 5)*controlPoints[3] + 70*pow(u, 4)*pow(1-u, 4)*controlPoints[4] + 56*pow(u, 5)*pow(1-u, 3)*controlPoints[5] + 28*pow(u, 6)*pow(1-u, 2)*controlPoints[6] + 8*pow(u, 7)*(1-u)*controlPoints[7] + pow(u, 8)*controlPoints[8];
            break;
        }

        case 9:{
            return pow(1-u, 9)*controlPoints[0] + 9*u*pow(1-u, 8)*controlPoints[1] + 36*pow(u, 2)*pow(1-u, 7)*controlPoints[2] + 84*pow(u, 3)*pow(1-u, 6)*controlPoints[3] + 126*pow(u, 4)*pow(1-u, 5)*controlPoints[4] + 126*pow(u, 5)*pow(1-u, 4)*controlPoints[5] + 84*pow(u, 6)*pow(1-u, 3)*controlPoints[6] + 36*pow(u, 7)*pow(1-u, 2)*controlPoints[7] + 9*pow(u, 8)*(1-u)*controlPoints[8] + pow(u, 9)*controlPoints[9];
            break;
        }
    }

    ROS_ERROR("[REGULATOR]: FATAL ERROR!!");
    exit(-1);
}

template <typename T>
T evaluateBezierFromPoint(T initialState, T finalState, float totalTime, float actualTime){
    if(actualTime < 0.0){
        actualTime = 0.0;
    }

    if(actualTime > totalTime){
        actualTime = totalTime;
    }

    float u = actualTime/totalTime;
    return pow(1-u, 7)*initialState + 7*u*pow(1-u, 6)*initialState + 21*pow(u, 2)*pow(1-u, 5)*initialState + 35*pow(u, 3)*pow(1-u, 4)*initialState
            + 35*pow(u, 4)*pow(1-u, 3)*finalState + 21*pow(u, 5)*pow(1-u, 2)*finalState + 7*pow(u, 6)*(1-u)*finalState + pow(u, 7)*finalState;
}

template <typename T>
T evaluateBezierDerivativeFromPoint(T initialState, T finalState, float totalTime, float actualTime, uint d){
    if(actualTime < 0.0){
        actualTime = 0.0;
    }

    if(actualTime > totalTime){
        actualTime = totalTime;
    }

    std::vector<T> controlPoints;
    controlPoints.insert(controlPoints.end(), 4, initialState);
    controlPoints.insert(controlPoints.end(), 4, finalState);
    for(uint ii = 0; ii < d; ii++){
        std::vector<T> controlPoints_tmp;
        
        for(uint jj = 0; jj < controlPoints.size()-1; jj++){
            T actualPoint = ((7-ii)/totalTime)*(controlPoints[jj+1] - controlPoints[jj]);
            controlPoints_tmp.push_back(actualPoint);
        }

        controlPoints.clear();
        controlPoints.shrink_to_fit();
        controlPoints = controlPoints_tmp;
    }

    float u = actualTime/totalTime;
    return evaluateBezier_general<T>(controlPoints, u);
}

template <typename T>
T evaluateBezier(std::vector<T> controlPoints, float totalTime, float actualTime, bool invertedTime){
    if(invertedTime){
        actualTime = totalTime - actualTime;
    }

    if(actualTime < 0.0){
        actualTime = 0.0;
    }

    if(actualTime > totalTime){
        actualTime = totalTime;
    }

    float u = actualTime/totalTime;
    return evaluateBezier_general<T>(controlPoints, u);
}

template <typename T>
T evaluateBezierDerivative(std::vector<T> controlPoints, float totalTime, float actualTime, bool invertedTime, uint d){
    if(invertedTime){
        actualTime = totalTime - actualTime;
    }

    if(actualTime < 0.0){
        actualTime = 0.0;
    }

    if(actualTime > totalTime){
        actualTime = totalTime;
    }

    std::vector<T> controlPoints_d = controlPoints;
    for(uint ii = 0; ii < d; ii++){
        std::vector<T> controlPoints_tmp;
        
        for(uint jj = 0; jj < controlPoints_d.size()-1; jj++){
            T actualPoint = ((7-ii)/totalTime)*(controlPoints_d[jj+1] - controlPoints_d[jj]);
            controlPoints_tmp.push_back(actualPoint);
        }

        controlPoints_d.clear();
        controlPoints_d.shrink_to_fit();
        controlPoints_d = controlPoints_tmp;
    }

    float u = actualTime/totalTime;
    return evaluateBezier_general<T>(controlPoints_d, u);
}

template <typename T>
T evaluateBezier_timeScaled(std::vector<T> controlPoints, std::vector<float> timePoints, float totalTime, float actualTime, bool invertedTime){
    if(invertedTime){
        actualTime = totalTime - actualTime;
    }
    
    if(actualTime < 0.0){
        actualTime = 0.0;
    }

    if(actualTime > totalTime){
        actualTime = totalTime;
    }

    float u = actualTime/totalTime;
    return evaluateBezier_general<T>(controlPoints, evaluateBezier_general<float>(timePoints, u));
}

template <typename T>
T evaluateBezierDerivative_timeScaled(std::vector<T> controlPoints, std::vector<float> timePoints, float totalTime, float actualTime, bool invertedTime, uint d){
    if(invertedTime){
        actualTime = totalTime - actualTime;
    }

    if(actualTime < 0.0){
        actualTime = 0.0;
    }

    if(actualTime > totalTime){
        actualTime = totalTime;
    }

    float u = actualTime/totalTime;
    float u_ = evaluateBezier_general<float>(timePoints, u);
    switch(d){
        case 1:{
            T dy = evaluateBezierDerivative<T>(controlPoints, 1.0, u_, false, 1);
            float du = evaluateBezierDerivative<float>(timePoints, 1.0, u, false, 1);
            return dy*du;

            break;
        }

        case 2:{
            T dy = evaluateBezierDerivative<T>(controlPoints, 1.0, u_, false, 1);
            T ddy = evaluateBezierDerivative<T>(controlPoints, 1.0, u_, false, 2);
            float du = evaluateBezierDerivative<float>(timePoints, 1.0, u, false, 1);
            float ddu = evaluateBezierDerivative<float>(timePoints, 1.0, u, false, 2);
            return ddy*du*du + dy*ddu;

            break;
        }

        case 3:{
            T dy = evaluateBezierDerivative<T>(controlPoints, 1.0, u_, false, 1);
            T ddy = evaluateBezierDerivative<T>(controlPoints, 1.0, u_, false, 2);
            T d3y = evaluateBezierDerivative<T>(controlPoints, 1.0, u_, false, 3);
            float du = evaluateBezierDerivative<float>(timePoints, 1.0, u, false, 1);
            float ddu = evaluateBezierDerivative<float>(timePoints, 1.0, u, false, 2);
            float d3u = evaluateBezierDerivative<float>(timePoints, 1.0, u, false, 3);
            return d3y*du*du*du + 3*ddy*du*ddu + dy*d3u;

            break;
        }

        case 4:{
            T dy = evaluateBezierDerivative<T>(controlPoints, 1.0, u_, false, 1);
            T ddy = evaluateBezierDerivative<T>(controlPoints, 1.0, u_, false, 2);
            T d3y = evaluateBezierDerivative<T>(controlPoints, 1.0, u_, false, 3);
            T d4y = evaluateBezierDerivative<T>(controlPoints, 1.0, u_, false, 4);
            float du = evaluateBezierDerivative<float>(timePoints, 1.0, u, false, 1);
            float ddu = evaluateBezierDerivative<float>(timePoints, 1.0, u, false, 2);
            float d3u = evaluateBezierDerivative<float>(timePoints, 1.0, u, false, 3);
            float d4u = evaluateBezierDerivative<float>(timePoints, 1.0, u, false, 4);
            return d4y*du*du*du*du + 6*d3y*du*du*ddu + ddy*(4*du*d3u + 3*ddu*ddu) + dy*d4u;

            break;
        }
    }

    ROS_ERROR("[REGULATOR]: FATAL ERROR!!");
    exit(-1);
}

template <typename T>
void degreeElevation(std::vector<T> controlPoints, std::vector<T>& controlPoints_new, uint d){
    uint r = d - (controlPoints.size()-1);

    if(r < 0){
        ROS_WARN("[REGULATOR]: Order Already Higher!");
        controlPoints_new = controlPoints;
        return;
    }

    controlPoints_new.resize(d+1);
    for(uint ii = 0; ii < controlPoints_new.size(); ii++){
        controlPoints_new[ii] = 0;

        for(uint kk = max<int>(0, ii-r); kk <= min<int>(controlPoints.size()-1, ii); kk++){
            float ff = bin(r, ii-kk)*bin(controlPoints.size()-1, kk)/bin(d, ii);
            controlPoints_new[ii] += ff*controlPoints[kk+1];
        }        
    }
}

template <typename T>
T evaluateBSpline(std::vector<T> controlPoints, std::vector<float> knot, double tt, int order){
    double tr = 0.0;
    if(tt >= knot[knot.size()-1] - 1e-3){
        tr = knot[knot.size()-1] - 1e-3;
    } else{
        tr = tt;
    }

    T result;
    for(uint ii = 0; ii < controlPoints.size(); ii++){
        double B = deBoor(order, ii, tr, knot);

        if(ii == 0){
			result = controlPoints[ii]*B;
		} else{
			result += controlPoints[ii]*B;
		}
    }

    return result;
}

template <typename T>
T evaluateBSplineDerivative(std::vector<T> controlPoints, std::vector<float> knot, double tt, int order, int derivate){
    std::vector<float> knot_d;
    knot_d.resize(knot.size() - 2);
    for(uint ii = 1; ii < knot.size() - 1; ii++)
        knot_d[ii-1] = knot[ii];

    std::vector<T> controlPoints_d;
    controlPoints_d.resize(controlPoints.size() - 1);
    for(uint ii = 0; ii < controlPoints.size() - 1; ii++){
        double deltaKnot = knot[ii+order+1] - knot[ii+1];
        controlPoints_d[ii] = (controlPoints[ii+1] - controlPoints[ii])*(order/deltaKnot);
    }

    if(derivate-1 == 0){
        return evaluateBSpline(controlPoints_d, knot_d, tt, order-1);
    } else{
        return evaluateBSplineDerivative(controlPoints_d, knot_d, tt, order-1, derivate-1);
    }
}