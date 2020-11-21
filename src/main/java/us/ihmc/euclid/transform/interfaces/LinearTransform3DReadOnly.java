package us.ihmc.euclid.transform.interfaces;

import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public interface LinearTransform3DReadOnly extends Matrix3DReadOnly
{
   QuaternionReadOnly getAsQuaternion();

   QuaternionReadOnly getPreScaleQuaternion();

   Vector3DReadOnly getScaleVector();

   QuaternionReadOnly getPostScaleQuaternion();

   default void getOrientation(Orientation3DBasics orientationToPack)
   {
      orientationToPack.set(getAsQuaternion());
   }

   default void getRotationVector(Vector3DBasics rotationVector)
   {
      getAsQuaternion().getRotationVector(rotationVector);
   }

   default void getEuler(Tuple3DBasics eulerAnglesToPack)
   {
      getAsQuaternion().getEuler(eulerAnglesToPack);
   }

   default double getScaleX()
   {
      return getScaleVector().getX();
   }

   default double getScaleY()
   {
      return getScaleVector().getY();
   }

   default double getScaleZ()
   {
      return getScaleVector().getZ();
   }

   default void transform(Orientation3DReadOnly orientationOriginal, Orientation3DBasics orientationTransformed)
   {
      getAsQuaternion().inverseTransform(orientationOriginal, orientationTransformed);
   }

   default void inverseTransform(Orientation3DReadOnly orientationOriginal, Orientation3DBasics orientationTransformed)
   {
      getAsQuaternion().inverseTransform(orientationOriginal, orientationTransformed);
   }
}