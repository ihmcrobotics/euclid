package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.polytope.interfaces.FrameConvexPolytope3DReadOnly;
import us.ihmc.euclid.referenceFrame.polytope.interfaces.FrameFace3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.RampPolytope3DView;

/**
 * Provides a {@link FrameConvexPolytope3DReadOnly} view backed by a {@link FrameRamp3DReadOnly}.
 * <p>
 * The implementation is expected to always reflect the current state of the ramp and its geometry
 * components are expected to be expressed in the global coordinate system of the box, i.e.
 * accounting for the box's pose.
 * </p>
 * <p>
 * This polytope reference frame is linked to the ramp reference frame.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public interface FrameRampPolytope3DView extends RampPolytope3DView, FrameConvexPolytope3DReadOnly
{
   /** {@inheritDoc} */
   @Override
   FrameFace3DReadOnly getRampFace();

   /** {@inheritDoc} */
   @Override
   FrameFace3DReadOnly getXMaxFace();

   /** {@inheritDoc} */
   @Override
   FrameFace3DReadOnly getYMaxFace();

   /** {@inheritDoc} */
   @Override
   default FrameFace3DReadOnly getMaxFace(Axis3D axis)
   {
      return (FrameFace3DReadOnly) RampPolytope3DView.super.getMaxFace(axis);
   }

   /** {@inheritDoc} */
   @Override
   FrameFace3DReadOnly getYMinFace();

   /** {@inheritDoc} */
   @Override
   FrameFace3DReadOnly getZMinFace();

   /** {@inheritDoc} */
   @Override
   default FrameFace3DReadOnly getMinFace(Axis3D axis)
   {
      return (FrameFace3DReadOnly) RampPolytope3DView.super.getMinFace(axis);
   }

   /** {@inheritDoc} */
   @Override
   FrameRamp3DReadOnly getOwner();

   /** {@inheritDoc} */
   @Override
   default FixedFrameRamp3DBasics copy()
   {
      return getOwner().copy();
   }

   /** {@inheritDoc} */
   @Override
   default FrameBoundingBox3DReadOnly getBoundingBox()
   {
      return getOwner().getBoundingBox();
   }

   /** {@inheritDoc} */
   @Override
   default FramePoint3DReadOnly getCentroid()
   {
      return getOwner().getCentroid();
   }
}
