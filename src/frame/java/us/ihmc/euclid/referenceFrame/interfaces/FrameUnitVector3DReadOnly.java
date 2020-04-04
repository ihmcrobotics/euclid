package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;

/**
 * Read-only interface for 3 dimensional unit-length vector expressed in given reference frame.
 * <p>
 * This unit vector shares the same API as a regular vector 3D while ensuring it is normalized when
 * accessing directly or indirectly its individual components, i.e. when invoking either
 * {@link #getX()}, {@link #getY()}, or {@link #getZ()}.
 * </p>
 * <p>
 * When the values of this vector are set to zero, the next time it is normalized it will be reset
 * to (1.0, 0.0, 0.0).
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public interface FrameUnitVector3DReadOnly extends FrameVector3DReadOnly, UnitVector3DReadOnly
{
}
