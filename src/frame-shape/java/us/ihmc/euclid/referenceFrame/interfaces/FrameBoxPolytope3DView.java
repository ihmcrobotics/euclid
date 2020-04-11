package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.polytope.interfaces.FrameConvexPolytope3DReadOnly;
import us.ihmc.euclid.referenceFrame.polytope.interfaces.FrameFace3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.BoxPolytope3DView;

/**
 * Provides a {@link FrameConvexPolytope3DReadOnly} view backed by a {@link FrameBox3DReadOnly}.
 * <p>
 * The implementation is expected to always reflect the current state of the box and its geometry
 * components are expected to be expressed in the global coordinate system of the box, i.e.
 * accounting for the box's pose.
 * </p>
 * <p>
 * This polytope reference frame is linked to the box reference frame.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public interface FrameBoxPolytope3DView extends BoxPolytope3DView, FrameConvexPolytope3DReadOnly
{
   /** {@inheritDoc} */
   @Override
   FrameFace3DReadOnly getXMaxFace();

   /** {@inheritDoc} */
   @Override
   FrameFace3DReadOnly getYMaxFace();

   /** {@inheritDoc} */
   @Override
   FrameFace3DReadOnly getZMaxFace();

   /** {@inheritDoc} */
   @Override
   default FrameFace3DReadOnly getMaxFace(Axis3D axis)
   {
      return (FrameFace3DReadOnly) BoxPolytope3DView.super.getMaxFace(axis);
   }

   /** {@inheritDoc} */
   @Override
   FrameFace3DReadOnly getXMinFace();

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
      return (FrameFace3DReadOnly) BoxPolytope3DView.super.getMinFace(axis);
   }

   /** {@inheritDoc} */
   @Override
   FrameBox3DReadOnly getOwner();

   /** {@inheritDoc} */
   @Override
   default FixedFrameBox3DBasics copy()
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
