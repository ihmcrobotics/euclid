package us.ihmc.euclid.referenceFrame.collision.interfaces;

import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.shape.collision.interfaces.EuclidShape3DCollisionResultBasics;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;

/**
 * Write and read interface adding {@code ReferenceFrame} support to
 * {@link EuclidShape3DCollisionResultBasics}.
 *
 * @author Sylvain Bertrand
 */
public interface EuclidFrameShape3DCollisionResultBasics extends EuclidFrameShape3DCollisionResultReadOnly, EuclidShape3DCollisionResultBasics
{
   /** {@inheritDoc} */
   @Override
   FramePoint3DBasics getPointOnA();

   /** {@inheritDoc} */
   @Override
   FramePoint3DBasics getPointOnB();

   /** {@inheritDoc} */
   @Override
   FrameVector3DBasics getNormalOnA();

   /** {@inheritDoc} */
   @Override
   FrameVector3DBasics getNormalOnB();

   /**
    * {@inheritDoc}
    *
    * @throws IllegalArgumentException if the argument does not implement {@link FrameShape3DReadOnly}
    */
   @Override
   default void setShapeA(Shape3DReadOnly shapeA)
   {
      if (shapeA == null || shapeA instanceof FrameShape3DReadOnly)
         setFrameShapeA((FrameShape3DReadOnly) shapeA);
      else
         throw new IllegalArgumentException("This collision result only supports frame shapes.");
   }

   /**
    * {@inheritDoc}
    *
    * @throws IllegalArgumentException if the argument does not implement {@link FrameShape3DReadOnly}
    */
   @Override
   default void setShapeB(Shape3DReadOnly shapeB)
   {
      if (shapeB == null || shapeB instanceof FrameShape3DReadOnly)
         setFrameShapeB((FrameShape3DReadOnly) shapeB);
      else
         throw new IllegalArgumentException("This collision result only supports frame shapes.");
   }

   /**
    * Sets the reference to the first shape.
    *
    * @param shapeA the new first shape in the collision result. Not modified, reference saved.
    */
   void setFrameShapeA(FrameShape3DReadOnly shapeA);

   /**
    * Sets the reference to the second shape.
    *
    * @param shapeB the new second shape in the collision result. Not modified, reference saved.
    */
   void setFrameShapeB(FrameShape3DReadOnly shapeB);

   /**
    * Copies the values from {@code other} into {@code this}.
    *
    * @param other the other object to copy the values from. Not modified.
    */
   default void set(EuclidFrameShape3DCollisionResultReadOnly other)
   {
      setFrameShapeA(other.getShapeA());
      setFrameShapeB(other.getShapeB());
      setShapesAreColliding(other.areShapesColliding());
      setSignedDistance(other.getSignedDistance());
      getPointOnA().setIncludingFrame(other.getPointOnA());
      getNormalOnA().setIncludingFrame(other.getNormalOnA());
      getPointOnB().setIncludingFrame(other.getPointOnB());
      getNormalOnB().setIncludingFrame(other.getNormalOnB());
   }
}
