package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.BoundingBox3DBasics;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeIOTools;
import us.ihmc.euclid.shape.primitives.interfaces.PointShape3DReadOnly;

/**
 * Read-only interface for a point shape 3D expressed in a given reference frame.
 * <p>
 * A point shape 3D is represented by its position.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface FramePointShape3DReadOnly extends PointShape3DReadOnly, FrameShape3DReadOnly, FramePoint3DReadOnly
{
   /**
    * {@inheritDoc}
    * <p>
    * Note that the centroid is also the position of this point shape.
    * </p>
    */
   @Override
   default FramePoint3DReadOnly getCentroid()
   {
      return this;
   }

   /** {@inheritDoc} */
   @Override
   default double distance(FramePoint3DReadOnly point)
   {
      return FramePoint3DReadOnly.super.distance(point);
   }

   /** {@inheritDoc} */
   @Override
   default void getBoundingBox(BoundingBox3DBasics boundingBoxToPack)
   {
      FrameShape3DReadOnly.super.getBoundingBox(boundingBoxToPack);
   }

   /** {@inheritDoc} */
   @Override
   default void getBoundingBox(ReferenceFrame destinationFrame, BoundingBox3DBasics boundingBoxToPack)
   {
      boundingBoxToPack.getMinPoint().set(this);
      getReferenceFrame().transformFromThisToDesiredFrame(destinationFrame, boundingBoxToPack.getMinPoint());
      boundingBoxToPack.getMaxPoint().set(boundingBoxToPack.getMinPoint());
   }

   /**
    * Returns {@code null} as this shape is not defined by a pose.
    */
   @Override
   default FrameShape3DPoseReadOnly getPose()
   {
      return null;
   }

   /** {@inheritDoc} */
   @Override
   FixedFramePointShape3DBasics copy();

   /**
    * Provides a {@code String} representation of this point shape 3D as follows:
    *
    * <pre>
    * Point shape 3D: (-0.362, -0.617,  0.066 ) - worldFrame
    * </pre>
    *
    * @return the {@code String} representing this point shape 3D.
    */
   @Override
   default String toString(String format)
   {
      return EuclidFrameShapeIOTools.getFramePointShape3DString(this);
   }
}
