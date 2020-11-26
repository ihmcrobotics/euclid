package us.ihmc.euclid.matrix.interfaces;

import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tools.SingularValueDecomposition3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

/**
 * Read-only interface used for a 3-by-3 linear transform.
 * <p>
 * A linear transform matrix behaves mostly like a regular 3D matrix. In addition to the base
 * features from {@link Matrix3DReadOnly}, the linear transform can be decomposed, using a singular
 * value decomposition, into:
 * 
 * <pre>
 * A = U W V
 * </pre>
 * 
 * where:
 * <ul>
 * <li><tt>A</tt> is this 3D linear transform.
 * <li><tt>U</tt> is the 3D pre-scale rotation.
 * <li><tt>W</tt> is the 3D scale.
 * <li><tt>U</tt> is the 3D post-scale rotation.
 * </ul>
 * </p>
 * 
 * @see SingularValueDecomposition3D
 * @author Sylvain Bertrand
 */
public interface LinearTransform3DReadOnly extends Matrix3DReadOnly
{
   /**
    * Returns the read-only view of this linear transform as a pure orientation.
    * <p>
    * The orientation represents the same transformation as {@code this} with the scales set to 1. The
    * returned quaternion is linked to this transform, i.e. it is automatically updated when this
    * transform is modified.
    * </p>
    * 
    * @return the read-only view of this linear transform as a pure orientation.
    */
   QuaternionReadOnly getAsQuaternion();

   /**
    * Returns the read-only view of the pre-scale rotation part of this linear transform.
    * <p>
    * The linear transform is decomposed, using a singular value decomposition, into:
    * 
    * <pre>
    * A = U W V
    * </pre>
    * 
    * where:
    * <ul>
    * <li><tt>A</tt> is this 3D linear transform.
    * <li><tt>U</tt> is the 3D pre-scale rotation.
    * <li><tt>W</tt> is the 3D scale.
    * <li><tt>U</tt> is the 3D post-scale rotation.
    * </ul>
    * The returned quaternion is linked to this transform, i.e. it is automatically updated when this
    * transform is modified.
    * </p>
    * 
    * @return the read-only view of the pre-scale rotation part of this linear transform.
    */
   QuaternionReadOnly getPreScaleQuaternion();

   /**
    * Returns the read-only view of the scale part of this linear transform.
    * <p>
    * The linear transform is decomposed, using a singular value decomposition, into:
    * 
    * <pre>
    * A = U W V
    * </pre>
    * 
    * where:
    * <ul>
    * <li><tt>A</tt> is this 3D linear transform.
    * <li><tt>U</tt> is the 3D pre-scale rotation.
    * <li><tt>W</tt> is the 3D scale.
    * <li><tt>U</tt> is the 3D post-scale rotation.
    * </ul>
    * The returned vector is linked to this transform, i.e. it is automatically updated when this
    * transform is modified.
    * </p>
    * 
    * @return the read-only view of the scale part of this linear transform.
    */
   Vector3DReadOnly getScaleVector();

   /**
    * Returns the read-only view of the post-scale rotation part of this linear transform.
    * <p>
    * The linear transform is decomposed, using a singular value decomposition, into:
    * 
    * <pre>
    * A = U W V
    * </pre>
    * 
    * where:
    * <ul>
    * <li><tt>A</tt> is this 3D linear transform.
    * <li><tt>U</tt> is the 3D pre-scale rotation.
    * <li><tt>W</tt> is the 3D scale.
    * <li><tt>U</tt> is the 3D post-scale rotation.
    * </ul>
    * The returned quaternion is linked to this transform, i.e. it is automatically updated when this
    * transform is modified.
    * </p>
    * 
    * @return the read-only view of the post-scale rotation part of this linear transform.
    */
   QuaternionReadOnly getPostScaleQuaternion();

   /**
    * Gets this linear transform stripped of its scale part into a pure orientation.
    * 
    * @param orientationToPack the object to store the orientation. Modified.
    */
   default void getOrientation(Orientation3DBasics orientationToPack)
   {
      orientationToPack.set(getAsQuaternion());
   }

   /**
    * Gets this linear transform stripped of its scale part, converts it, and stores it as a rotation
    * vector.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation. A
    * rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle of the
    * same axis-angle.
    * </p>
    * 
    * @param rotationVector the vector to store the rotation. Modified.
    */
   default void getRotationVector(Vector3DBasics rotationVector)
   {
      getAsQuaternion().getRotationVector(rotationVector);
   }

   /**
    * Gets this linear transform stripped of its scale part, converts it, and stores it as Euler
    * angles.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    * 
    * @param eulerAnglesToPack the tuple in which the Euler angles are stored. Modified.
    */
   default void getEuler(Tuple3DBasics eulerAnglesToPack)
   {
      getAsQuaternion().getEuler(eulerAnglesToPack);
   }

   /**
    * Gets the x-component of this linear transform's scale.
    * <p>
    * The linear transform is decomposed, using a singular value decomposition, into:
    * 
    * <pre>
    * A = U W V
    * </pre>
    * 
    * where:
    * <ul>
    * <li><tt>A</tt> is this 3D linear transform.
    * <li><tt>U</tt> is the 3D pre-scale rotation.
    * <li><tt>W</tt> is the 3D scale.
    * <li><tt>U</tt> is the 3D post-scale rotation.
    * </ul>
    * </p>
    * 
    * @return the x-component of this linear transform's scale.
    */
   default double getScaleX()
   {
      return getScaleVector().getX();
   }

   /**
    * Gets the y-component of this linear transform's scale.
    * <p>
    * The linear transform is decomposed, using a singular value decomposition, into:
    * 
    * <pre>
    * A = U W V
    * </pre>
    * 
    * where:
    * <ul>
    * <li><tt>A</tt> is this 3D linear transform.
    * <li><tt>U</tt> is the 3D pre-scale rotation.
    * <li><tt>W</tt> is the 3D scale.
    * <li><tt>U</tt> is the 3D post-scale rotation.
    * </ul>
    * </p>
    * 
    * @return the y-component of this linear transform's scale.
    */
   default double getScaleY()
   {
      return getScaleVector().getY();
   }

   /**
    * Gets the z-component of this linear transform's scale.
    * <p>
    * The linear transform is decomposed, using a singular value decomposition, into:
    * 
    * <pre>
    * A = U W V
    * </pre>
    * 
    * where:
    * <ul>
    * <li><tt>A</tt> is this 3D linear transform.
    * <li><tt>U</tt> is the 3D pre-scale rotation.
    * <li><tt>W</tt> is the 3D scale.
    * <li><tt>U</tt> is the 3D post-scale rotation.
    * </ul>
    * </p>
    * 
    * @return the z-component of this linear transform's scale.
    */
   default double getScaleZ()
   {
      return getScaleVector().getZ();
   }

   /**
    * Transforms the given {@code orientationToTransform} by this linear transform stripped of its
    * scale part.
    * <p>
    * The operation is equivalent to {@code this.getAsQuaternion().transform(orientationToTransform)}.
    * </p>
    *
    * @param orientationToTransform the orientation to be transformed. Modified.
    */
   default void transform(Orientation3DBasics orientationToTransform)
   {
      transform(orientationToTransform, orientationToTransform);
   }

   /**
    * Transforms the given {@code orientationOriginal} and stores the result in
    * {@code orientationTransformed}.
    * <p>
    * The operation is equivalent to
    * {@code this.getAsQuaternion().transform(orientationOriginal, orientationTransformed)}.
    * </p>
    *
    * @param orientationOriginal    the original value of the orientation to be transformed. Not
    *                               modified.
    * @param orientationTransformed the result of the original orientation after transformation.
    *                               Modified.
    */
   default void transform(Orientation3DReadOnly orientationOriginal, Orientation3DBasics orientationTransformed)
   {
      getAsQuaternion().transform(orientationOriginal, orientationTransformed);
   }

   /**
    * Performs the inverse of the transform to the given orientation by this linear transform stripped
    * of its scale part.
    * <p>
    * The operation is equivalent to
    * {@code this.getAsQuaternion().inverseTransform(orientationToTransform)}.
    * </p>
    *
    * @param orientationToTransform the orientation to be transformed. Modified.
    */
   default void inverseTransform(Orientation3DBasics orientationToTransform)
   {
      inverseTransform(orientationToTransform, orientationToTransform);
   }

   /**
    * Performs the inverse of the transform to the given {@code orientationOriginal} and stores the
    * result in {@code orientationTransformed}.
    * <p>
    * The operation is equivalent to
    * {@code this.getAsQuaternion().inverseTransform(orientationOriginal, orientationTransformed)}.
    * </p>
    *
    * @param orientationOriginal    the original value of the orientation to be transformed. Not
    *                               modified.
    * @param orientationTransformed the result of the original orientation after transformation.
    *                               Modified.
    */
   default void inverseTransform(Orientation3DReadOnly orientationOriginal, Orientation3DBasics orientationTransformed)
   {
      getAsQuaternion().inverseTransform(orientationOriginal, orientationTransformed);
   }
}