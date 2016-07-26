/**
* Software License Agreement (BSD License)
*
*           Copyright (C) 2015 by Israel D. Gebru,
*           Perception Team, INRIA-Grenoble, France
*                   All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* The views and conclusions contained in the software and documentation are those
* of the authors and should not be interpreted as representing official policies,
* either expressed or implied, of the FreeBSD Project.
*/
#ifndef FEATURES_H
#define FEATURES_H

#include "Sample.h"

#include <eigen2/Eigen/Core>
#include <vector>

class Features
{
public:
	Features();

	inline const Eigen::VectorXd& Eval(const Sample& s) const
	{
		const_cast<Features*>(this)->UpdateFeatureVector(s);
		return m_featVec;
	}

	virtual void Eval(const MultiSample& s, std::vector<Eigen::VectorXd>& featVecs)
	{
		// default implementation
		featVecs.resize(s.GetRects().size());
		for (int i = 0; i < (int)featVecs.size(); ++i)
		{
			featVecs[i] = Eval(s.GetSample(i));
		}
	}

	inline int GetCount() const { return m_featureCount; }

protected:

	int m_featureCount;
	Eigen::VectorXd m_featVec;

	void SetCount(int c);
	virtual void UpdateFeatureVector(const Sample& s) = 0;

};

#endif
