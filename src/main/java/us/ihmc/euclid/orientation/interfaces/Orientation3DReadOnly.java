package us.ihmc.euclid.orientation.interfaces;

import us.ihmc.euclid.axisAngle.interfaces.AxisAngleBasics;
import us.ihmc.euclid.exceptions.NotAnOrientation2DException;
import us.ihmc.euclid.interfaces.EuclidGeometry;
import us.ihmc.euclid.matrix.interfaces.CommonMatrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixBasics;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
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
public interface Orientation3DReadOnly extends EuclidGeometry
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
   void get(CommonMatrix3DBasics rotationMatrixToPack);

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
    * Calculates and returns the angular distance from origin.
    *
    * @return the angle from origin in range: [0, 2<i>pi</i>].
    */
   default double angle()
   {
      return angle(false);
   }

   /**
    * Calculates and returns the angular distance from origin.
    *
    * @param limitToPi Limits the result to [0, <i>pi</i>].
    * @return the angle from origin in range: [0, 2<i>pi</i>].
    */
   double angle(boolean limitToPi);

   /**
    * Calculates and returns the angular distance between this(self) and other orientation.
    *
    * @param other the other orientation to be compared to. Not modified.
    * @return the angle between the two orientations. The result is not guaranteed to be in [0,
    *         <i>pi</i>].
    */
   default double distance(Orientation3DReadOnly other)
   {
      return distance(other, false);
   }

   /**
    * Calculates and returns the angular distance between this(self) and other orientation.
    *
    * @param other     the other orientation to be compared to. Not modified.
    * @param limitToPi Limits the result to [0, <i>pi</i>pi].
    * @return the angle between the two orientations. The result is not guaranteed to be in [0,
    *         <i>pi</i>].
    */
   double distance(Orientation3DReadOnly other, boolean limitToPi);

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
    * Tests if {@code this} and {@code other} represent the same orientation to an {@code epsilon}.
    * <p>
    * Note that {@code this.geometricallyEquals(other, epsilon) == true} does not necessarily imply
    * that the 2 orientations are of the same type nor that they are equal on a per-component bases.
    * </p>
    *
    * @param object  the object to compare against this. Not modified.
    * @param epsilon the maximum angle for the two orientations to be considered equal.
    * @return {@code true} if the two orientations represent the same geometry, {@code false}
    *         otherwise.
    */
   @Override
   default boolean geometricallyEquals(Object object, double epsilon)
   {
      if (!(object instanceof Orientation3DReadOnly))
         return false;
      if (epsilon >= Math.PI)
         return true; // Trivial case. If epsilon is greater than pi, then any pair of orientations are equal.
      Orientation3DReadOnly other = (Orientation3DReadOnly) object;
      return distance(other, true) <= epsilon;
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
