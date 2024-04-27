#include "IKSim.h"

IKSim::IKSim(const std::string& name, BaseSystem* target) :
	BaseSimulator(name),
	m_object(target)
{
	fileLoaded = false;
	hermite_t = 0;
	delta_t = 0;
	prev_t = 0;
	initial = 0.01;
	increment = 0.01;
}

void IKSim::setHermite(HermiteSystem* target)
{
	hermite = target;
}

int IKSim::init(double time)
{
	return 0;
}

int IKSim::step(double time)
{
	delta_t = time - prev_t;

	// reposition target to start when at end of spline
	if (hermite_t > 0.990)
	{
		hermite_t = 0;
		transition = true;
		initial = 0.1;
		increment = 0.002;
	}

	if (fileLoaded)
	{
		Vector currentP, targetP, error;

		// get bob's current hand position
		m_object->getState(currentP);

		// get target position based on hermite t value
		LineSegment target = hermite->getLookUpTable(hermite_t);
		//setVector(targetP, target[0], target[1], target[2]);

		// calculate error and set new target position if bob is close
		VecSubtract(error, target.start, currentP);

		if (VecLength(error) < 0.15)
		{
			hermite_t += 0.00015;
		}

		if (hermite_t < 0.00005)
		{
			hermite_t = 0;
			if (transition)
			{
				VecCopy(pTargetP, currentP);
				VecCopy(velocity, error);
				transition = false;
			}
			Vector velocity_med;
			VecCopy(velocity_med, velocity);
			VecScale(velocity_med, initial);
			Vector pTarget;
			VecAdd(pTarget, pTargetP, velocity_med);
			m_object->setState(pTarget);
			if (initial < 1)
				initial += increment;
		}
		else {
			m_object->setState(target.start);

		}
	}

	prev_t = time;
	return 0;

}

int IKSim::command(int argc, myCONST_SPEC char** argv)
{
	if (argc < 1)
	{
		animTcl::OutputMessage("simulator %s: wrong number of params.", m_name.c_str());
		return TCL_ERROR;
	}
	else if (strcmp(argv[0], "read") == 0)
	{
		if (argc == 2)
		{
			hermite->loadFile(argv[1]); //这块还没有输入文件，这两个程序输入文件的格式是一样的，所以这块不需要转换
			hermite->generateLookUpTable();//这里生成完之后就是step里面的代码了，再之后就是myscene里面的代码了
			animTcl::OutputMessage("[PseudoInverseIK] Read spline from file");
			fileLoaded = true;

			// Reset LERP variables
			hermite_t = 0;
			transition = true;
			initial = 0.1;
			increment = 0.01;

			param[0] = 1; // at 0 position, 1 for pseudo, 2 for CCD
			param[1] = 1; // at 1 position, 1 for pseudo, 2 for CCD
			m_object->setState(param);
			param[0] = 0; // 0 for not updating resting position
		}
		else if (argc == 3 && strcmp(argv[2], "CCDIK") == 0) { // simulator iksim read animation2.txt CCDIK
			hermite->loadFile(argv[1]);
			hermite->generateLookUpTable();
			animTcl::OutputMessage("[CCDIK] Read spline from file");
			fileLoaded = true;

			// Reset LERP variables
			hermite_t = 0;
			transition = true;
			initial = 0.1;
			increment = 0.03;

			param[1] = 2; // at 1 position, 1 for pseudo, 2 for CCD
			param[0] = 2; // 2 for updating resting position and it is CCD
			// Calcalue the pos of each joint
			m_object->setState(param);
			param[0] = 0; // 0 for not updating resting position
		}
		else
		{
			animTcl::OutputMessage("[iksim] Usage: simulator iksim read spline.txt");
			return TCL_ERROR;
		}
	}

	glutPostRedisplay();
	return TCL_OK;
}