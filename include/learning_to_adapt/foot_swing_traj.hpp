#pragma once


class FootSwingTraj {
private:
    Eigen::Vector3d startPos_, endPos_;
    double height_;
    Eigen::Vector3d interpPos_, interpVel_, interpAcc_;

public:
    explicit FootSwingTraj() {
        startPos_.setZero();
        endPos_.setZero();
        height_ = 0.;
        interpPos_.setZero();
        interpVel_.setZero();
        interpAcc_.setZero();
    }
    
    void setStartPosition(const Eigen::Ref<const Eigen::Vector3d>& startPos) {
        startPos_ = startPos;
        interpPos_ = startPos;
    }
    
    void setEndPosition(const Eigen::Ref<const Eigen::Vector3d>& endPos) {
        endPos_ = endPos;
    }
    
    void setHeight(const double height) {
        height_ = height;
    }
    
    void computeSwingTrajectory(const double inputPhase, const double swingTime) {
        interpPos_ = cubicBezier(inputPhase, startPos_, endPos_);
        //TODO: if needed add vel and acc here
        
        if (inputPhase < 0.5) {
            interpPos_(2) = cubicBezier(inputPhase*2., startPos_(2), startPos_(2) + height_);
        }
        else {
            interpPos_(2) = cubicBezier(inputPhase*2.-1., startPos_(2) + height_, endPos_(2));
        }
    }
    
    const Eigen::Vector3d& getPosition() {
        return interpPos_;
    }
    
    void reset() {
        startPos_.setZero();
        endPos_.setZero();
        height_ = 0.;
        interpPos_.setZero();
        interpVel_.setZero();
        interpAcc_.setZero();
    }
    
    // Bezier Stuff //
    double cubicBezier(double inputPhase, double startPos, double endPos) {
        assert(inputPhase >= 0 && inputPhase <= 1);
        double bezier = std::pow(inputPhase, 3.) + 3. * (std::pow(inputPhase, 2.) * (1. - inputPhase));
        return startPos + bezier * (endPos - startPos);
    }
    
    Eigen::Vector3d cubicBezier(double inputPhase, const Eigen::Ref<const Eigen::Vector3d>& startPos, const Eigen::Ref<const Eigen::Vector3d>& endPos) {
        assert(inputPhase >= 0 && inputPhase <= 1);
        double bezier = std::pow(inputPhase, 3.) + 3. * (std::pow(inputPhase, 2.) * (1. - inputPhase));
        return startPos + bezier * (endPos - startPos);
    }
    
    double cubicBezierFirstDerivative(double inputPhase, double startPos, double endPos) {
        assert(inputPhase >= 0 && inputPhase <= 1);
        double bezierFirstDerivative = 6. * inputPhase * (1. - inputPhase);
        return bezierFirstDerivative * (endPos - startPos);
    }
    
    Eigen::Vector3d cubicBezierFirstDerivative(double inputPhase, const Eigen::Ref<const Eigen::Vector3d>& startPos, const Eigen::Ref<const Eigen::Vector3d>& endPos) {
        assert(inputPhase >= 0 && inputPhase <= 1);
        double bezierFirstDerivative = 6. * inputPhase * (1. - inputPhase);
        return bezierFirstDerivative * (endPos - startPos);
    }
    
    double cubicBezierSecondDerivative(double inputPhase, double startPos, double endPos) {
        assert(inputPhase >= 0 && inputPhase <= 1);
        double bezierSecondDerivative = 6. - 12. * inputPhase;
        return bezierSecondDerivative * (endPos - startPos);
    }
    
    Eigen::Vector3d cubicBezierSecondDerivative(double inputPhase, const Eigen::Ref<const Eigen::Vector3d>& startPos, const Eigen::Ref<const Eigen::Vector3d>& endPos)
    {
        assert(inputPhase >= 0 && inputPhase <= 1);
        double bezierSecondDerivative = 6. - 12. * inputPhase;
        return bezierSecondDerivative * (endPos - startPos);
    }
};

