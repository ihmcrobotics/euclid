package us.ihmc.euclid.orientation.interfaces;

import us.ihmc.euclid.axisAngle.interfaces.AxisAngleBasics;
import us.ihmc.euclid.exceptions.NotAnOrientation2DException;
import us.ihmc.euclid.matrix.RotationScaleMatrix;
import us.ihmc.euclid.matrix.interfaces.*;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.RotationMatrixTools;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DBasics;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DReadOnly;
import us.ihmc.euclid.yawPitchRoll.interfaces.YawPitchRollBasics;

/**
 * Base interface to easily identify implementations that represent a physical orientation in 3
 * dimensions.
 * <p>
 * Even though the representation used is unknown at this level of abstraction, this interface
 * allows to enforce a minimum set of features that all representations of an orientation should
 * provide, such as transformation functions.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface Orientation3DReadOnly
{
   /**
    * Default tolerance to use when testing if this orientation represents an orientation in the
    * XY-plane.
    */
   static final double ORIENTATION_2D_EPSILON = 1.0e-8;
   /**
    * Default tolerance used when testing if this is a zero orientation.
    */
   static final double ZERO_EPSILON = 1.0e-8;

   /**
    * Tests if at least one of this orientation's component contains {@link Double#NaN}.
    *
    * @return if this orientation has at least one component being {@link Double#NaN}.
    */
   boolean containsNaN();

   /**
    * Test if this orientation 3D represents a zero orientation.
    * <p>
    * A zero orientation when used as a transform, leaves a the transformed geometry unchanged.
    * </p>
    * <p>
    * Equivalent to calling {@link #isZeroOrientation(double)} with {@link #ZERO_EPSILON}.
    * </p>
    *
    * @param epsilon the tolerance used for the test.
    * @return {@code true} if this is equal to a zero orientation, {@code false} otherwise.
    */
   default boolean isZeroOrientation()
   {
      return isZeroOrientation(ZERO_EPSILON);
   }

   /**
    * Test if this orientation 3D represents a zero orientation.
    * <p>
    * A zero orientation when used as a transform, leaves a the transformed geometry unchanged.
    * </p>
    *
    * @param epsilon the tolerance used for the test.
    * @return {@code true} if this is equal to a zero orientation, {@code false} otherwise.
    */
   boolean isZeroOrientation(double epsilon);

   /**
    * Tests if this orientation 3D actually represents a rotation strictly around the z-axis.
    * <p>
    * This is commonly used to test if this orientation can be used to transform 2D geometry object.
    * </p>
    * <p>
    * This test uses the default tolerance {@link #ORIENTATION_2D_EPSILON}, to specify explicitly the
    * tolerance to be use, see {@link #isOrientation2D(double)}.
    * </p>
    *
    * @return {@code true} if this orientation represents a 2D orientation in the XY-plane,
    *         {@code false} otherwise.
    */
   default boolean isOrientation2D()
   {
      return isOrientation2D(ORIENTATION_2D_EPSILON);
   }

   /**
    * Tests if this orientation 3D actually represents a rotation strictly around the z-axis.
    * <p>
    * This is commonly used to test if this orientation can be used to transform 2D geometry object.
    * </p>
    * <p>
    * The implementation of this test depends on the type of representation used for this orientation.
    * </p>
    *
    * @param epsilon the tolerance to use.
    * @return {@code true} if this orientation represents a 2D orientation in the XY-plane,
    *         {@code false} otherwise.
    */
   boolean isOrientation2D(double epsilon);

   /**
    * Tests if this orientation 3D actually represents a rotation strictly around the z-axis.
    * <p>
    * This is commonly used to test if this orientation can be used to transform 2D geometry object.
    * </p>
    *
    * @throws NotAnOrientation2DException if this orientation does not represent a rotation strictly
    *                                     around the z-axis.
    */
   default void checkIfOrientation2D()
   {
      checkIfOrientation2D(ORIENTATION_2D_EPSILON);
   }

   /**
    * Tests if this orientation 3D actually represents a rotation strictly around the z-axis.
    * <p>
    * This is commonly used to test if this orientation can be used to transform 2D geometry object.
    * </p>
    *
    * @param epsilon the tolerance to use.
    * @throws NotAnOrientation2DException if this orientation does not represent a rotation strictly
    *                                     around the z-axis.
    */
   default void checkIfOrientation2D(double epsilon)
   {
      if (!isOrientation2D(epsilon))
         throw new NotAnOrientation2DException(this);
   }

   /**
    * Converts, if necessary, and packs this orientation into a 3-by-3 rotation matrix.
    *
    * @param rotationMatrixToPack the rotation matrix into which this orientation is to be stored.
    *                             Modified.
    */
   void get(RotationMatrixBasics rotationMatrixToPack);

   /**
    * Converts, if necessary, and packs this orientation into an axis-angle.
    *
    * @param axisAngleToPack the axis-angle into which this orientation is to be stored. Modified.
    */
   void get(AxisAngleBasics axisAngleToPack);

   /**
    * Converts, if necessary, and packs this orientation in a quaternion.
    *
    * @param quaternionToPack the quaternion into which this orientation is to be stored. Modified.
    */
   void get(QuaternionBasics quaternionToPack);

   /**
    * Converts, if necessary, and packs this orientation in a yaw-pitch-roll.
    *
    * @param yawPitchRollToPack the yaw-pitch-roll into which this orientation is to be stored.
    *                           Modified.
    */
   void get(YawPitchRollBasics yawPitchRollToPack);

   /**
    * Converts and packs this orientation in a 3D rotation vector.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation. A
    * rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle of the
    * same axis-angle.
    * </p>
    *
    * @param rotationVectorToPack the rotation vector in which this orientation is to be stored.
    *                             Modified.
    */
   void getRotationVector(Vector3DBasics rotationVectorToPack);

   /**
    * Converts and packs this orientation in a yaw-pitch-roll representation.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    * <p>
    * The yaw-pitch-roll representation describes a 3D orientation as a succession of three rotations
    * around three axes:
    * <ol>
    * <li>yaw: rotation around the z-axis,
    * <li>pitch: rotation around the y-axis,
    * <li>roll: rotation around the x-axis.
    * </ol>
    * </p>
    * <p>
    * As an example, a rotation matrix can be computed from a yaw-pitch-roll representation as follows:
    *
    * <pre>
    *     / cos(yaw) -sin(yaw) 0 \   /  cos(pitch) 0 sin(pitch) \   / 1     0          0     \
    * R = | sin(yaw)  cos(yaw) 0 | * |      0      1     0      | * | 0 cos(roll) -sin(roll) |
    *     \    0         0     1 /   \ -sin(pitch) 0 cos(pitch) /   \ 0 sin(roll)  cos(roll) /
    * </pre>
    * </p>
    *
    * @param yawPitchRollToPack the array in which the yaw-pitch-roll angles are stored. Modified.
    * @deprecated Use {@link #get(YawPitchRollBasics)} instead.
    */
   @Deprecated
   void getYawPitchRoll(double[] yawPitchRollToPack);

   /**
    * Computes and packs the orientation described by this orientation as the Euler angles.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @param eulerAnglesToPack the tuple in which the Euler angles are stored. Modified.
    */
   void getEuler(Tuple3DBasics eulerAnglesToPack);

   /**
    * Computes and returns the yaw angle from the yaw-pitch-roll representation of this orientation.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @return the yaw angle around the z-axis.
    */
   double getYaw();

   /**
    * Computes and returns the pitch angle from the yaw-pitch-roll representation of this orientation.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @return the pitch angle around the y-axis.
    */
   double getPitch();

   /**
    * Computes and returns the roll angle from the yaw-pitch-roll representation of this orientation.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @return the roll angle around the x-axis.
    */
   double getRoll();

   /**
    * Transforms the given tuple by this orientation.
    * <p>
    * If the given tuple is expressed in the local frame described by this orientation, then the tuple
    * is transformed such that it is, after this method is called, expressed in the base frame in which
    * this orientation is expressed.
    * </p>
    *
    * @param tupleToTransform the 3D tuple to be transformed. Modified.
    */
   default void transform(Tuple3DBasics tupleToTransform)
   {
      transform(tupleToTransform, tupleToTransform);
   }

   /**
    * Transforms the tuple {@code tupleOriginal} by this orientation and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * If the given tuple is expressed in the local frame described by this orientation, then the tuple
    * is transformed such that it is, after this method is called, expressed in the base frame in which
    * this orientation is expressed.
    * </p>
    *
    * @param tupleOriginal    the original value of the tuple to be transformed. Not modified.
    * @param tupleTransformed the result of the original tuple after transformation. Modified.
    */
   void transform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed);

   /**
    * Transforms the given tuple by this orientation and adds the result to the tuple.
    * <p>
    * If the given tuple is expressed in the local frame described by this orientation, then the tuple
    * is transformed such that it is, after this method is called, expressed in the base frame in which
    * this orientation is expressed.
    * </p>
    *
    * @param tupleToTransform the 3D tuple to be transformed. Modified.
    */
   default void addTransform(Tuple3DBasics tupleToTransform)
   {
      addTransform(tupleToTransform, tupleToTransform);
   }

   /**
    * Transforms the tuple {@code tupleOriginal} by this orientation and <b>adds</b> the result to
    * {@code tupleTransformed}.
    * <p>
    * If the given tuple is expressed in the local frame described by this orientation, then the tuple
    * is transformed such that it is, after this method is called, expressed in the base frame in which
    * this orientation is expressed.
    * </p>
    *
    * @param tupleOriginal    the original value of the tuple to be transformed. Not modified.
    * @param tupleTransformed the result of the original tuple after transformation. Modified.
    */
   default void addTransform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      double x = tupleTransformed.getX();
      double y = tupleTransformed.getY();
      double z = tupleTransformed.getZ();
      transform(tupleOriginal, tupleTransformed);
      tupleTransformed.add(x, y, z);
   }

   /**
    * Transforms the given tuple by this orientation and subtracts the result to the tuple.
    * <p>
    * If the given tuple is expressed in the local frame described by this orientation, then the tuple
    * is transformed such that it is, after this method is called, expressed in the base frame in which
    * this orientation is expressed.
    * </p>
    *
    * @param tupleToTransform the 3D tuple to be transformed. Modified.
    */
   default void subTransform(Tuple3DBasics tupleToTransform)
   {
      subTransform(tupleToTransform, tupleToTransform);
   }

   /**
    * Transforms the tuple {@code tupleOriginal} by this orientation and <b>subtracts</b> the result to
    * {@code tupleTransformed}.
    * <p>
    * If the given tuple is expressed in the local frame described by this orientation, then the tuple
    * is transformed such that it is, after this method is called, expressed in the base frame in which
    * this orientation is expressed.
    * </p>
    *
    * @param tupleOriginal    the original value of the tuple to be transformed. Not modified.
    * @param tupleTransformed the result of the original tuple after transformation. Modified.
    */
   default void subTransform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      double x = tupleTransformed.getX();
      double y = tupleTransformed.getY();
      double z = tupleTransformed.getZ();
      transform(tupleOriginal, tupleTransformed);
      tupleTransformed.sub(x, y, z);
      tupleTransformed.negate();
   }

   /**
    * Transforms the given tuple by this orientation.
    * <p>
    * If the given tuple is expressed in the local frame described by this orientation, then the tuple
    * is transformed such that it is, after this method is called, expressed in the base frame in which
    * this orientation is expressed.
    * </p>
    *
    * @param tupleToTransform the 2D tuple to be transformed. Modified.
    * @throws NotAnOrientation2DException if this orientation is not a 2D orientation.
    */
   default void transform(Tuple2DBasics tupleToTransform)
   {
      transform(tupleToTransform, tupleToTransform, true);
   }

   /**
    * Transforms the tuple {@code tupleOriginal} by this orientation and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * If the given tuple is expressed in the local frame described by this orientation, then the tuple
    * is transformed such that it is, after this method is called, expressed in the base frame in which
    * this orientation is expressed.
    * </p>
    *
    * @param tupleOriginal    the original value of the tuple to be transformed. Not modified.
    * @param tupleTransformed the result of the original tuple after transformation. Modified.
    * @throws NotAnOrientation2DException if this orientation is not a 2D orientation.
    */
   default void transform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed)
   {
      transform(tupleOriginal, tupleTransformed, true);
   }

   /**
    * Transforms the given tuple by this orientation.
    * <p>
    * If the given tuple is expressed in the local frame described by this orientation, then the tuple
    * is transformed such that it is, after this method is called, expressed in the base frame in which
    * this orientation is expressed.
    * </p>
    *
    * @param checkIfOrientation2D whether this method should assert that this orientation represents a
    *                             transformation in the XY plane.
    * @param tupleToTransform     the 2D tuple to be transformed. Modified.
    * @throws NotAnOrientation2DException if this orientation is not a 2D orientation.
    */
   default void transform(Tuple2DBasics tupleToTransform, boolean checkIfOrientation2D)
   {
      transform(tupleToTransform, tupleToTransform, checkIfOrientation2D);
   }

   /**
    * Transforms the tuple {@code tupleOriginal} by this orientation and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * If the given tuple is expressed in the local frame described by this orientation, then the tuple
    * is transformed such that it is, after this method is called, expressed in the base frame in which
    * this orientation is expressed.
    * </p>
    *
    * @param checkIfOrientation2D whether this method should assert that this orientation represents a
    *                             transformation in the XY plane.
    * @param tupleOriginal        the original value of the tuple to be transformed. Not modified.
    * @param tupleTransformed     the result of the original tuple after transformation. Modified.
    * @throws NotAnOrientation2DException if this orientation is not a 2D orientation.
    */
   void transform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfOrientation2D);

   /**
    * Transforms the given matrix by this orientation.
    * <p>
    * If the given matrix is expressed in the local frame described by this orientation, then the
    * matrix is transformed such that it is, after this method is called, expressed in the base frame
    * in which this orientation is expressed.
    * </p>
    *
    * @param matrixToTransform the 3D matrix to be transformed. Modified.
    */
   default void transform(Matrix3DBasics matrixToTransform)
   {
      transform(matrixToTransform, matrixToTransform);
   }

   /**
    * Transforms the matrix {@code matrixOriginal} by this orientation and stores the result in
    * {@code matrixTransformed}.
    * <p>
    * If the given matrix is expressed in the local frame described by this orientation, then the
    * matrix is transformed such that it is, after this method is called, expressed in the base frame
    * in which this orientation is expressed.
    * </p>
    *
    * @param matrixOriginal    the original value of the matrix to be transformed. Not modified.
    * @param matrixTransformed the result of the original matrix after transformation. Modified.
    */
   void transform(Matrix3DReadOnly matrixOriginal, Matrix3DBasics matrixTransformed);

   /**
    * Transforms the vector part, i.e. the {@code x}, {@code y}, and {@code z} components, of the given
    * 4D vector, its scalar component {@code s} remains unaffected by this operation.
    * <p>
    * If the given vector part is expressed in the local frame described by this orientation, then the
    * vector part is transformed such that it is, after this method is called, expressed in the base
    * frame in which this orientation is expressed.
    * </p>
    *
    * @param vectorToTransform the 4D tuple to be transformed. Modified.
    */
   default void transform(Vector4DBasics vectorToTransform)
   {
      transform(vectorToTransform, vectorToTransform);
   }

   /**
    * Transforms the vector part, i.e. the {@code x}, {@code y}, and {@code z} components, of the given
    * {@code vectorOriginal} and stores the result in {@code vectorTransformed}.
    * <p>
    * The scalar component {@code s} remains unaffected by this operation and is simply copied over.
    * </p>
    * <p>
    * If the given vector part is expressed in the local frame described by this orientation, then the
    * vector part is transformed such that it is, after this method is called, expressed in the base
    * frame in which this orientation is expressed.
    * </p>
    *
    * @param vectorOriginal    the original value of the vector to be transformed. Not modified.
    * @param vectorTransformed the result of the original vector after transformation. Modified.
    */
   void transform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed);

   /**
    * Transforms the given rotation matrix by this orientation.
    * <p>
    * The operation is equivalent to prepend this orientation to the given rotation matrix.
    * </p>
    *
    * @param matrixToTransform the rotation matrix to be transformed. Modified.
    */
   default void transform(RotationMatrixBasics matrixToTransform)
   {
      transform(matrixToTransform, matrixToTransform);
   }

   /**
    * Transforms the given {@code matrixOriginal} and stores the result in {@code matrixTransformed}.
    * <p>
    * The operation is equivalent to prepend this orientation to the {@code matrixOriginal} and store
    * the result in {@code matrixTransformed}.
    * </p>
    *
    * @param matrixOriginal    the original value of the matrix to be transformed. Not modified.
    * @param matrixTransformed the result of the original matrix after transformation. Modified.
    */
   default void transform(RotationMatrixReadOnly matrixOriginal, RotationMatrixBasics matrixTransformed)
   {
      RotationMatrixTools.multiply(this, false, matrixOriginal, false, matrixTransformed);
   }

   /**
    * Transforms the given rotation-scale matrix by this orientation.
    * <p>
    * The operation is equivalent to prepend this orientation to the rotation part of the given
    * rotation-scale matrix.
    * </p>
    *
    * @param matrixToTransform the rotation-scale matrix to be transformed. Modified.
    */
   default void transform(RotationScaleMatrix matrixToTransform)
   {
      transform(matrixToTransform.getRotationMatrix());
   }

   /**
    * Transforms the given {@code matrixOriginal} and stores the result in {@code matrixTransformed}.
    * <p>
    * The operation is equivalent to prepend this orientation to the rotation part of
    * {@code matrixOriginal} and store the result in {@code matrixTransformed}.
    * </p>
    *
    * @param matrixOriginal    the original value of the matrix to be transformed. Not modified.
    * @param matrixTransformed the result of the original matrix after transformation. Modified.
    */
   default void transform(RotationScaleMatrixReadOnly matrixOriginal, RotationScaleMatrix matrixTransformed)
   {
      matrixTransformed.set(matrixOriginal);
      transform(matrixOriginal.getRotationMatrix(), matrixTransformed.getRotationMatrix());
   }

   /**
    * Transforms the given {@code orientationToTransform} by this orientation.
    * <p>
    * The operation is equivalent to prepend this orientation to the given
    * {@code orientationToTransform}.
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
    * The operation is equivalent to prepend this orientation to the {@code orientationOriginal} and
    * store the result in {@code orientationTransformed}.
    * </p>
    *
    * @param orientationOriginal    the original value of the orientation to be transformed. Not
    *                               modified.
    * @param orientationTransformed the result of the original orientation after transformation.
    *                               Modified.
    */
   default void transform(Orientation3DReadOnly orientationOriginal, Orientation3DBasics orientationTransformed)
   {
      if (orientationTransformed != orientationOriginal)
         orientationTransformed.set(orientationOriginal);
      orientationTransformed.prepend(this);
   }

   /**
    * Performs the inverse of the transform to the given tuple by this orientation.
    * <p>
    * If the given tuple is expressed in the base frame in which this orientation is expressed, then
    * the tuple is transformed such that it is, after this method is called, expressed in the local
    * frame described by this orientation.
    * </p>
    *
    * @param tupleToTransform the 3D tuple to be transformed. Modified.
    */
   default void inverseTransform(Tuple3DBasics tupleToTransform)
   {
      inverseTransform(tupleToTransform, tupleToTransform);
   }

   /**
    * Performs the inverse of the transform to the tuple {@code tupleOriginal} by this orientation and
    * stores the result in {@code tupleTransformed}.
    * <p>
    * If the given tuple is expressed in the base frame in which this orientation is expressed, then
    * the tuple is transformed such that it is, after this method is called, expressed in the local
    * frame described by this orientation.
    * </p>
    *
    * @param tupleOriginal    the original value of the tuple to be transformed. Not modified.
    * @param tupleTransformed the result of the original tuple after transformation. Modified.
    */
   void inverseTransform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed);

   /**
    * Performs the inverse of the transform to the given tuple by this orientation.
    * <p>
    * If the given tuple is expressed in the base frame in which this orientation is expressed, then
    * the tuple is transformed such that it is, after this method is called, expressed in the local
    * frame described by this orientation.
    * </p>
    *
    * @param tupleToTransform the 2D tuple to be transformed. Modified.
    * @throws NotAnOrientation2DException if this orientation is not a 2D orientation.
    */
   default void inverseTransform(Tuple2DBasics tupleToTransform)
   {
      inverseTransform(tupleToTransform, tupleToTransform, true);
   }

   /**
    * Performs the inverse of the transform to the tuple {@code tupleOriginal} by this orientation and
    * stores the result in {@code tupleTransformed}.
    * <p>
    * If the given tuple is expressed in the base frame in which this orientation is expressed, then
    * the tuple is transformed such that it is, after this method is called, expressed in the local
    * frame described by this orientation.
    * </p>
    *
    * @param tupleOriginal    the original value of the tuple to be transformed. Not modified.
    * @param tupleTransformed the result of the original tuple after transformation. Modified.
    * @throws NotAnOrientation2DException if this orientation is not a 2D orientation.
    */
   default void inverseTransform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed)
   {
      inverseTransform(tupleOriginal, tupleTransformed, true);
   }

   /**
    * Performs the inverse of the transform to the given tuple by this orientation.
    * <p>
    * If the given tuple is expressed in the base frame in which this orientation is expressed, then
    * the tuple is transformed such that it is, after this method is called, expressed in the local
    * frame described by this orientation.
    * </p>
    *
    * @param checkIfOrientation2D whether this method should assert that this orientation represents a
    *                             transformation in the XY plane.
    * @param tupleToTransform     the 2D tuple to be transformed. Modified.
    * @throws NotAnOrientation2DException if this orientation is not a 2D orientation.
    */
   default void inverseTransform(Tuple2DBasics tupleToTransform, boolean checkIfOrientation2D)
   {
      inverseTransform(tupleToTransform, tupleToTransform, checkIfOrientation2D);
   }

   /**
    * Performs the inverse of the transform to the tuple {@code tupleOriginal} by this orientation and
    * stores the result in {@code tupleTransformed}.
    * <p>
    * If the given tuple is expressed in the base frame in which this orientation is expressed, then
    * the tuple is transformed such that it is, after this method is called, expressed in the local
    * frame described by this orientation.
    * </p>
    *
    * @param checkIfOrientation2D whether this method should assert that this orientation represents a
    *                             transformation in the XY plane.
    * @param tupleOriginal        the original value of the tuple to be transformed. Not modified.
    * @param tupleTransformed     the result of the original tuple after transformation. Modified.
    * @throws NotAnOrientation2DException if this orientation is not a 2D orientation.
    */
   void inverseTransform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfOrientation2D);

   /**
    * Performs the inverse of the transform to the given matrix by this orientation.
    * <p>
    * If the given matrix is expressed in the base frame in which this orientation is expressed, then
    * the matrix is transformed such that it is, after this method is called, expressed in the local
    * frame described by this orientation.
    * </p>
    *
    * @param matrixToTransform the 3D matrix to be transformed. Modified.
    */
   default void inverseTransform(Matrix3DBasics matrixToTransform)
   {
      inverseTransform(matrixToTransform, matrixToTransform);
   }

   /**
    * Performs the inverse of the transform to the matrix {@code matrixOriginal} by this orientation
    * and stores the result in {@code matrixTransformed}.
    * <p>
    * If the given matrix is expressed in the base frame in which this orientation is expressed, then
    * the matrix is transformed such that it is, after this method is called, expressed in the local
    * frame described by this orientation.
    * </p>
    *
    * @param matrixOriginal    the original value of the matrix to be transformed. Not modified.
    * @param matrixTransformed the result of the original matrix after transformation. Modified.
    */
   void inverseTransform(Matrix3DReadOnly matrixOriginal, Matrix3DBasics matrixTransformed);

   /**
    * Performs the inverse of the transform to the vector part, i.e. the {@code x}, {@code y}, and
    * {@code z} components, of the given 4D vector, its scalar component {@code s} remains unaffected
    * by this operation.
    * <p>
    * If the given vector part is expressed in the base frame in which this orientation is expressed,
    * then the vector part is transformed such that it is, after this method is called, expressed in
    * the local frame described by this orientation.
    * </p>
    *
    * @param vectorToTransform the 4D tuple to be transformed. Modified.
    */
   default void inverseTransform(Vector4DBasics vectorToTransform)
   {
      inverseTransform(vectorToTransform, vectorToTransform);
   }

   /**
    * Performs the inverse of the transform to the vector part, i.e. the {@code x}, {@code y}, and
    * {@code z} components, of the given {@code vectorOriginal} and stores the result in
    * {@code vectorTransformed}.
    * <p>
    * The scalar component {@code s} remains unaffected by this operation and is simply copied over.
    * </p>
    * <p>
    * If the given vector part is expressed in the local frame described by this orientation, then the
    * vector part is transformed such that it is, after this method is called, expressed in the base
    * frame in which this orientation is expressed.
    * </p>
    *
    * @param vectorOriginal    the original value of the vector to be transformed. Not modified.
    * @param vectorTransformed the result of the original vector after transformation. Modified.
    */
   void inverseTransform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed);

   /**
    * Performs the inverse of the transform to the given rotation matrix by this orientation.
    * <p>
    * The operation is equivalent to prepend the inverse of this orientation to the given rotation
    * matrix.
    * </p>
    *
    * @param matrixToTransform the rotation matrix to be transformed. Modified.
    */
   default void inverseTransform(RotationMatrixBasics matrixToTransform)
   {
      inverseTransform(matrixToTransform, matrixToTransform);
   }

   /**
    * Performs the inverse of the transform to the given {@code matrixOriginal} and stores the result
    * in {@code matrixTransformed}.
    * <p>
    * The operation is equivalent to prepend the inverse of this orientation to the
    * {@code matrixOriginal} and store the result in {@code matrixTransformed}.
    * </p>
    *
    * @param matrixOriginal    the original value of the matrix to be transformed. Not modified.
    * @param matrixTransformed the result of the original matrix after transformation. Modified.
    */
   default void inverseTransform(RotationMatrixReadOnly matrixOriginal, RotationMatrixBasics matrixTransformed)
   {
      RotationMatrixTools.multiply(this, true, matrixOriginal, false, matrixTransformed);
   }

   /**
    * Performs the inverse of the transform to the given rotation-scale matrix by this orientation.
    * <p>
    * The operation is equivalent to prepend the inverse of this orientation to the rotation part of
    * the given rotation-scale matrix.
    * </p>
    *
    * @param matrixToTransform the rotation-scale matrix to be transformed. Modified.
    */
   default void inverseTransform(RotationScaleMatrix matrixToTransform)
   {
      inverseTransform(matrixToTransform.getRotationMatrix());
   }

   /**
    * Performs the inverse of the transform to the given {@code matrixOriginal} and stores the result
    * in {@code matrixTransformed}.
    * <p>
    * The operation is equivalent to prepend the inverse of this orientation to the rotation part of
    * {@code matrixOriginal} and store the result in {@code matrixTransformed}.
    * </p>
    *
    * @param matrixOriginal    the original value of the matrix to be transformed. Not modified.
    * @param matrixTransformed the result of the original matrix after transformation. Modified.
    */
   default void inverseTransform(RotationScaleMatrixReadOnly matrixOriginal, RotationScaleMatrix matrixTransformed)
   {
      matrixTransformed.set(matrixOriginal);
      inverseTransform(matrixOriginal.getRotationMatrix(), matrixTransformed.getRotationMatrix());
   }

   /**
    * Performs the inverse of the transform to the given orientation by this orientation.
    * <p>
    * The operation is equivalent to prepend the inverse of this orientation to the given orientation.
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
    * The operation is equivalent to prepend the inverse of this orientation to the
    * {@code orientationOriginal} and store the result in {@code orientationTransformed}.
    * </p>
    *
    * @param orientationOriginal    the original value of the orientation to be transformed. Not
    *                               modified.
    * @param orientationTransformed the result of the original orientation after transformation.
    *                               Modified.
    */
   default void inverseTransform(Orientation3DReadOnly orientationOriginal, Orientation3DBasics orientationTransformed)
   {
      if (orientationTransformed != orientationOriginal)
         orientationTransformed.set(orientationOriginal);
      orientationTransformed.prependInvertOther(this);
   }

   /**
    * Provides a {@code String} representation of this orientation converted to yaw-pitch-roll angles
    * as follows: yaw-pitch-roll: (yaw, pitch, roll).
    *
    * @return a string representation of this orientation 3D.
    */
   default String toStringAsYawPitchRoll()
   {
      return EuclidCoreIOTools.getStringAsYawPitchRoll(this);
   }
}
