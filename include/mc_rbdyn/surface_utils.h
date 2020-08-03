/*
 * Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#ifndef _H_MCRBDYNSURFACEUTILS_H_
#define _H_MCRBDYNSURFACEUTILS_H_

#include <mc_rbdyn/api.h>

#include <SpaceVecAlg/SpaceVecAlg>

namespace mc_rbdyn
{

/** Load surfaces from .rsdf file in the provided directory and add them to the
 * provided robot
 *
 * \param robot Robot where the surface are going to be loaded
 *
 * \param dirname Directory where .rsdf files are searched
 */
MC_RBDYN_DLLAPI std::vector<SurfacePtr> readRSDFFromDir(Robot & robot, std::string_view dirname);

/** Returns the intersection points when putting the two provided surfaces in contact.
 *
 * If the surfaces are compatible but the two surfaces don't intersect then s1.points() is returned
 *
 * \param s1 First contact surface
 *
 * \param s2 Second contact surface
 *
 * \returns Intersection points in s1 frame's parent body frame
 *
 * \throws If the two surfaces are not compatible for a contact
 */
MC_RBDYN_DLLAPI std::vector<sva::PTransformd> intersection(const Surface & s1, const Surface & s2);

} // namespace mc_rbdyn

#endif
