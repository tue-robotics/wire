/************************************************************************
 *  Copyright (C) 2012 Eindhoven University of Technology (TU/e).       *
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

#ifndef WM_EVIDENCE_H_
#define WM_EVIDENCE_H_

#include "wire/core/PropertySet.h"

namespace mhf {

/**
 * @author Sjoerd van den Dries
 * @date December, 2012
 * @version 1.0
 *
 * @brief The class Evidence represents a set of properties (PropertySet)
 * that all originate from one physical entity in the world from one
 * specific point in time.
 *
 * The class Evidence represents a set of properties (PropertySet)
 * that all originate from one physical entity in the world from one
 * specific point in time. For example, an Evidence object could
 * represent that a 5-cm small, green object has been observed at
 * position (0.5, 0.2, 0.8), with corresponding uncertainty. The
 * uncertainty is explicitly represented in the Evidence as a
 * probability density function per property (see Property).
 */
class Evidence : public PropertySet {
public:

    static int N_EVIDENCE;

    /**
     * @brief Evidence constructor
     * @param timestamp The time from which the evidence originates
     */
    Evidence(Time timestamp);

    virtual ~Evidence();    

protected:

    /// The time from which the evidence originates
    Time timestamp_;

};

}

#endif /* MEASUREMENT_H_ */
