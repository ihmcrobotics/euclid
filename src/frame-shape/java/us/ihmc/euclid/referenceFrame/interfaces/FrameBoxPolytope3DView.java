package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.polytope.interfaces.FrameConvexPolytope3DReadOnly;
import us.ihmc.euclid.referenceFrame.polytope.interfaces.FrameFace3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.BoxPolytope3DView;

public interface FrameBoxPolytope3DView extends BoxPolytope3DView, FrameConvexPolytope3DReadOnly
{
   FrameFace3DReadOnly getXMaxFace();

   FrameFace3DReadOnly getYMaxFace();

   FrameFace3DReadOnly getZMaxFace();

   default FrameFace3DReadOnly getMaxFace(Axis3D axis)
   {
      return (FrameFace3DReadOnly) BoxPolytope3DView.super.getMaxFace(axis);
   }

   FrameFace3DReadOnly getXMinFace();

   FrameFace3DReadOnly getYMinFace();

   FrameFace3DReadOnly getZMinFace();

   default FrameFace3DReadOnly getMinFace(Axis3D axis)
   {
      return (FrameFace3DReadOnly) BoxPolytope3DView.super.getMinFace(axis);
   }

   FrameBox3DReadOnly getOwner();

   @Override
   default FixedFrameShape3DBasics copy()
   {
      return getOwner().copy();
   }

   @Override
   default FrameBoundingBox3DReadOnly getBoundingBox()
   {
      return getOwner().getBoundingBox();
   }

   @Override
   default FramePoint3DReadOnly getCentroid()
   {
      return getOwner().getCentroid();
   }
}
