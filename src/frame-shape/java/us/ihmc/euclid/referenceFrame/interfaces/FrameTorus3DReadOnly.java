package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.BoundingBox3DBasics;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeIOTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeTools;
import us.ihmc.euclid.shape.primitives.interfaces.Torus3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;

/**
 * Read-only interface for a torus 3D expressed in given reference frame.
 * <p>
 * A torus is represented by its position, its axis of revolution, the radius of its tube, and the
 * radius from the torus axis to the tube center.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface FrameTorus3DReadOnly extends Torus3DReadOnly, FrameShape3DReadOnly
{
   /** {@inheritDoc} */
   @Override
   FramePoint3DReadOnly getPosition();

   /** {@inheritDoc} */
   @Override
   FrameUnitVector3DReadOnly getAxis();

   /**
    * {@inheritDoc}
    * <p>
    * Note that the centroid is also the position of this capsule.
    * </p>
    */
   @Override
   default FramePoint3DReadOnly getCentroid()
   {
      return getPosition();
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
      EuclidFrameShapeTools.boundingBoxTorus3D(this, destinationFrame, boundingBoxToPack);
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
   FixedFrameTorus3DBasics copy();

   /**
    * Gets a representative {@code String} of this frame capsule 3D given a specific format to use.
    * <p>
    * Using the default format {@link EuclidCoreIOTools#DEFAULT_FORMAT}, this provides a {@code String}
    * as follows:
    *
    * <pre>
    * Capsule 3D: [position: (-0.362, -0.617,  0.066 ), axis: ( 0.634, -0.551, -0.543 ), length:  0.170, radius:  0.906] - worldFrame
    * </pre>
    * </p>
    */
   @Override
   default String toString(String format)
   {
      return EuclidFrameShapeIOTools.getFrameTorus3DString(format, this);
   }
}
