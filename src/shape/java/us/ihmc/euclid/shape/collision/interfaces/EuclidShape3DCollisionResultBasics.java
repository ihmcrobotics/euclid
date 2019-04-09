package us.ihmc.euclid.shape.collision.interfaces;

import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

public interface EuclidShape3DCollisionResultBasics extends EuclidShape3DCollisionResultReadOnly, EuclidCollisionResultBasics
{
   void setShapeA(Shape3DReadOnly shapeA);

   void setShapeB(Shape3DReadOnly shapeB);

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

}
