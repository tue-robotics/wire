/************************************************************************
 *  Copyright (C) 2021 Eindhoven University of Technology (TU/e).       *
 *  All rights reserved.                                                *
 ************************************************************************
 *  Redistribution and use in source and binary forms, with or without  *
 *  modification, are permitted provided that the following conditions  *
 *  are met:                                                            *
 *                                                                      *
 *      1.  Redistributions of source code must retain the above        *
 *          copyright notice, this list of conditions and the following *
 *          disclaimer.                                                 *
 *                                                                      *
 *      2.  Redistributions in binary form must reproduce the above     *
 *          copyright notice, this list of conditions and the following *
 *          disclaimer in the documentation and/or other materials      *
 *          provided with the distribution.                             *
 *                                                                      *
 *  THIS SOFTWARE IS PROVIDED BY TU/e "AS IS" AND ANY EXPRESS OR        *
 *  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED      *
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE  *
 *  ARE DISCLAIMED. IN NO EVENT SHALL TU/e OR CONTRIBUTORS BE LIABLE    *
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR        *
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT   *
 *  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;     *
 *  OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF       *
 *  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT           *
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE   *
 *  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH    *
 *  DAMAGE.                                                             *
 *                                                                      *
 *  The views and conclusions contained in the software and             *
 *  documentation are those of the authors and should not be            *
 *  interpreted as representing official policies, either expressed or  *
 *  implied, of TU/e.                                                   *
 ************************************************************************/

#include "test_plugin.h"

TestPlugin::TestPlugin() : pdf_(nullptr), bool_param_(false), double_param_(0), string_param_() {
}

TestPlugin::TestPlugin(const pbl::PDF& pdf) : pdf_(pdf.clone()) {
}

TestPlugin::TestPlugin(const TestPlugin& orig) : mhf::IStateEstimator(orig), pdf_(nullptr),
    bool_param_(orig.bool_param_), double_param_(orig.double_param_), string_param_(orig.string_param_) {

    if (orig.pdf_) {
        pdf_ = orig.pdf_->clone();
    }
}

TestPlugin::~TestPlugin() {
    if (pdf_)
        delete pdf_;
}

TestPlugin* TestPlugin::clone() const {
    return new TestPlugin(*this);
}

void TestPlugin::update(const pbl::PDF& /*z*/, const mhf::Time& /*time*/) {
}

void TestPlugin::propagate(const mhf::Time& /*time*/) {
}

void TestPlugin::reset() {
}

const pbl::PDF& TestPlugin::getValue() const {
    return *pdf_;
}

bool TestPlugin::setParameter(const std::string& param, bool b) {
    if (param == "bool_param") {
        bool_param_ = b;
    } else {
        return false;
    }
    return true;
}

bool TestPlugin::setParameter(const std::string &param, double v) {
    if (param == "double_param") {
        double_param_ = v;
    } else {
        return false;
    }
    return true;
}

bool TestPlugin::setParameter(const std::string& param, const std::string& s) {
    if (param == "string_param") {
        string_param_ = s;
    } else {
        return false;
    }
    return true;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( TestPlugin, mhf::IStateEstimator )
