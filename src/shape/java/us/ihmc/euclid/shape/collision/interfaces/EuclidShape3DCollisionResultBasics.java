package us.ihmc.euclid.shape.collision.interfaces;

import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

/**
 * Write and read interface for holding the result of a collision query between two shapes.
 * 
 * @author Sylvain Bertrand
 */
public interface EuclidShape3DCollisionResultBasics extends EuclidShape3DCollisionResultReadOnly, Clearable, Transformable
{
   /**
    * Sets the reference to the first shape.
    * 
    * @param shapeA the new first shape in the collision result. Not modified, reference saved.
    */
   void setShapeA(Shape3DReadOnly shapeA);

   /**
    * Sets the reference to the second shape.
    * 
    * @param shapeB the new second shape in the collision result. Not modified, reference saved.
    */
   void setShapeB(Shape3DReadOnly shapeB);

   /** {@inheritDoc} */
   @Override
   Point3DBasics getPointOnA();

   /** {@inheritDoc} */
   @Override
   Point3DBasics getPointOnB();

   /** {@inheritDoc} */
   @Override
   Vector3DBasics getNormalOnA();

   /** {@inheritDoc} */
   @Override
   Vector3DBasics getNormalOnB();

   /**
    * Sets the collision state between the two shapes.
    * 
    * @param shapesAreColliding whether the shapes are colliding or not.
    */
   void setShapesAreColliding(boolean shapesAreColliding);

   /**
    * Sets the distance, i.e. either the separation distance or the penetration depth.
    * <p>
    * The distance should be signed as follows:
    * <ul>
    * <li>positive in the case the shapes are not colliding, in which case it represents the separating
    * distance.
    * <li>negative in the case the shapes are colliding, in which case it represents the depth of the
    * penetration.
    * </ul>
    * </p>
    * 
    * @param distance the value of the distance.
    */
   void setSignedDistance(double distance);

   /** {@inheritDoc} */
   @Override
   default boolean containsNaN()
   {
      return EuclidShape3DCollisionResultReadOnly.super.containsNaN();
   }

   /** {@inheritDoc} */
   @Override
   default void setToZero()
   {
      setShapesAreColliding(false);
      setSignedDistance(0.0);
      getPointOnA().setToZero();
      getNormalOnA().setToZero();
      getPointOnB().setToZero();
      getNormalOnB().setToZero();
   }

   /** {@inheritDoc} */
   @Override
   default void setToNaN()
   {
      setShapesAreColliding(false);
      setSignedDistance(Double.NaN);
      getPointOnA().setToNaN();
      getNormalOnA().setToNaN();
      getPointOnB().setToNaN();
      getNormalOnB().setToNaN();
   }

   /**
    * Swaps the two shapes and the collision properties.
    */
   default void swapShapes()
   {
      Shape3DReadOnly tempShape = getShapeA();
      setShapeA(getShapeB());
      setShapeB(tempShape);

      Point3DBasics pointOnA = getPointOnA();
      double tempX = pointOnA.getX();
      double tempY = pointOnA.getY();
      double tempZ = pointOnA.getZ();

      pointOnA.set(getPointOnB());
      getPointOnB().set(tempX, tempY, tempZ);

      Vector3DBasics normalOnA = getNormalOnA();
      tempX = normalOnA.getX();
      tempY = normalOnA.getY();
      tempZ = normalOnA.getZ();
      normalOnA.set(getNormalOnB());
      getNormalOnB().set(tempX, tempY, tempZ);
   }

   /** {@inheritDoc} */
   @Override
   default void applyTransform(Transform transform)
   {
      if (!getPointOnA().containsNaN())
         getPointOnA().applyTransform(transform);
      if (!getPointOnB().containsNaN())
         getPointOnB().applyTransform(transform);
      if (!getNormalOnA().containsNaN())
         getNormalOnA().applyTransform(transform);
      if (!getNormalOnB().containsNaN())
         getNormalOnB().applyTransform(transform);
   }

   /** {@inheritDoc} */
   @Override
   default void applyInverseTransform(Transform transform)
   {
      if (!getPointOnA().containsNaN())
         getPointOnA().applyInverseTransform(transform);
      if (!getPointOnB().containsNaN())
         getPointOnB().applyInverseTransform(transform);
      if (!getNormalOnA().containsNaN())
         getNormalOnA().applyInverseTransform(transform);
      if (!getNormalOnB().containsNaN())
         getNormalOnB().applyInverseTransform(transform);
   }
}
