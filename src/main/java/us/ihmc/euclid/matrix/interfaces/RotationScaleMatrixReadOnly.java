package us.ihmc.euclid.matrix.interfaces;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DBasics;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DReadOnly;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;

/**
 * Read interface for 3-by-3 rotation-scale matrices.
 * <p>
 * A rotation-scale matrix <i>M</i> is equal to: <i> M = R * S </i>. Where <i>R</i> is a rotation
 * matrix, and <i>S</i> is a scaling matrix as follows:
 *
 * <pre>
 *     / s<sub>x</sub> 0 0 \
 * <i>S</i> = | 0 s<sub>y</sub> 0 |
 *     \ 0 0 s<sub>z</sub> /
 * </pre>
 *
 * where s<sub>x</sub>, s<sub>y</sub>, and s<sub>z</sub> three non-zero positive scale factors.
 * </p>
 * <p>
 * Note: To conserve the form <i> M = R * S </i>, the algebra with a rotation-scale matrix is rather
 * restrictive. For instance, an rotation-scale matrix cannot be inverted. However, it can still
 * perform the inverse of the transform it represents on geometry objects.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface RotationScaleMatrixReadOnly extends Matrix3DReadOnly
{
   /**
    * Returns the read-only reference to the rotation matrix used to compose this rotation-scale
    * matrix.
    *
    * @return the read-only reference to the rotation matrix.
    */
   RotationMatrixReadOnly getRotationMatrix();

   /**
    * Returns the read-only reference to the scale factors used to compose this rotation-scale matrix.
    *
    * @return the read-only reference to the scale factors.
    */
   Vector3DReadOnly getScale();

   /**
    * Returns the current value of the first scale factor of this rotation-scale matrix.
    *
    * @return the value of the first scale factor.
    */
   default double getScaleX()
   {
      return getScale().getX();
   }

   /**
    * Returns the current value of the second scale factor of this rotation-scale matrix.
    *
    * @return the value of the second scale factor.
    */
   default double getScaleY()
   {
      return getScale().getY();
   }

   /**
    * Returns the current value of the third scale factor of this rotation-scale matrix.
    *
    * @return the value of the third scale factor.
    */
   default double getScaleZ()
   {
      return getScale().getZ();
   }

   /** {@inheritDoc} */
   @Override
   default double getM00()
   {
      return getRotationMatrix().getM00() * getScaleX();
   }

   /** {@inheritDoc} */
   @Override
   default double getM01()
   {
      return getRotationMatrix().getM01() * getScaleY();
   }

   /** {@inheritDoc} */
   @Override
   default double getM02()
   {
      return getRotationMatrix().getM02() * getScaleZ();
   }

   /** {@inheritDoc} */
   @Override
   default double getM10()
   {
      return getRotationMatrix().getM10() * getScaleX();
   }

   /** {@inheritDoc} */
   @Override
   default double getM11()
   {
      return getRotationMatrix().getM11() * getScaleY();
   }

   /** {@inheritDoc} */
   @Override
   default double getM12()
   {
      return getRotationMatrix().getM12() * getScaleZ();
   }

   /** {@inheritDoc} */
   @Override
   default double getM20()
   {
      return getRotationMatrix().getM20() * getScaleX();
   }

   /** {@inheritDoc} */
   @Override
   default double getM21()
   {
      return getRotationMatrix().getM21() * getScaleY();
   }

   /** {@inheritDoc} */
   @Override
   default double getM22()
   {
      return getRotationMatrix().getM22() * getScaleZ();
   }

   /**
    * Retrieves the scale factor with the maximum value and returns it.
    *
    * @return the maximum value among the scale factors.
    */
   default double getMaxScale()
   {
      return EuclidCoreTools.max(getScaleX(), getScaleY(), getScaleZ());
   }

   /**
    * Packs the rotation part.
    *
    * @param orientationToPack the orientation in which the rotation part is stored. Modified.
    */
   default void getRotation(Orientation3DBasics orientationToPack)
   {
      orientationToPack.set(getRotationMatrix());
   }

   /**
    * Packs the rotation part as a rotation matrix and stores it into a row-major 1D array.
    *
    * @param rotationMatrixArrayToPack the array in which the coefficients of the rotation part are
    *                                  stored. Modified.
    */
   default void getRotation(double[] rotationMatrixArrayToPack)
   {
      getRotationMatrix().get(rotationMatrixArrayToPack);
   }

   /**
    * Packs the rotation part as a rotation matrix.
    *
    * @param rotationMatrixToPack the rotation matrix in which the rotation part is stored. Modified.
    */
   default void getRotation(DenseMatrix64F rotationMatrixToPack)
   {
      getRotationMatrix().get(rotationMatrixToPack);
   }

   /**
    * Packs the rotation part as an rotation vector.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation. A
    * rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle of the
    * same axis-angle.
    * </p>
    *
    * @param rotationVectorToPack the rotation vector in which the rotation part is stored. Modified.
    */
   default void getRotation(Vector3DBasics rotationVectorToPack)
   {
      getRotationMatrix().getRotationVector(rotationVectorToPack);
   }

   /**
    * Packs the orientation described by the rotation part as the Euler angles.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @param eulerAnglesToPack the tuple in which the Euler angles are stored. Modified.
    */
   default void getRotationEuler(Tuple3DBasics eulerAnglesToPack)
   {
      getRotationMatrix().getEuler(eulerAnglesToPack);
   }

   /**
    * Packs the orientation described by the rotation part as the yaw-pitch-roll angles.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @param yawPitchRollToPack the array in which the yaw-pitch-roll angles are stored. Modified.
    * @deprecated Use {@link YawPitchRoll} with {@link #getRotationMatrix()}.
    */
   default void getRotationYawPitchRoll(double[] yawPitchRollToPack)
   {
      getRotationMatrix().getYawPitchRoll(yawPitchRollToPack);
   }

   /**
    * Computes and returns the yaw angle from the yaw-pitch-roll representation of the rotation part.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @return the yaw angle around the z-axis.
    */
   default double getRotationYaw()
   {
      return getRotationMatrix().getYaw();
   }

   /**
    * Computes and returns the pitch angle from the yaw-pitch-roll representation of the rotation part.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @return the pitch angle around the y-axis.
    */
   default double getRotationPitch()
   {
      return getRotationMatrix().getPitch();
   }

   /**
    * Computes and returns the roll angle from the yaw-pitch-roll representation of the rotation part.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @return the roll angle around the x-axis.
    */
   default double getRotationRoll()
   {
      return getRotationMatrix().getRoll();
   }

   /**
    * Packs the scale factors in a tuple.
    *
    * @param scaleToPack the tuple in which the scale factors are stored. Modified.
    */
   default void getScale(Tuple3DBasics scaleToPack)
   {
      scaleToPack.set(getScale());
   }

   /** {@inheritDoc} */
   @Override
   default void transform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      tupleTransformed.setX(getScaleX() * tupleOriginal.getX());
      tupleTransformed.setY(getScaleY() * tupleOriginal.getY());
      tupleTransformed.setZ(getScaleZ() * tupleOriginal.getZ());

      getRotationMatrix().transform(tupleTransformed, tupleTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void addTransform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      double x = tupleTransformed.getX();
      double y = tupleTransformed.getY();
      double z = tupleTransformed.getZ();
      transform(tupleOriginal, tupleTransformed);
      tupleTransformed.add(x, y, z);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      tupleTransformed.setX(getScaleX() * tupleOriginal.getX());
      tupleTransformed.setY(getScaleY() * tupleOriginal.getY());

      getRotationMatrix().transform(tupleTransformed, tupleTransformed, checkIfTransformInXYPlane);
   }

   /**
    * Transforms the given orientation by the rotation part of this rotation-scale matrix.
    * <p>
    * orientationToTransform = this.getRotationMatrix() * orientationToTransform <br>
    * </p>
    *
    * @param orientationToTransform the orientation to transform. Modified.
    */
   default void transform(Orientation3DBasics orientationToTransform)
   {
      transform(orientationToTransform, orientationToTransform);
   }

   /**
    * Transforms the given orientation {@code orientationOriginal} and stores the result into
    * {@code orientationTransformed}.
    * <p>
    * orientationToTransform = this.getRotationMatrix() * orientationOriginal <br>
    * </p>
    *
    * @param orientationOriginal    the orientation to transform. Not modified.
    * @param orientationTransformed the orientation in which the result is stored. Modified.
    */
   default void transform(Orientation3DReadOnly orientationOriginal, Orientation3DBasics orientationTransformed)
   {
      getRotationMatrix().transform(orientationOriginal, orientationTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      vectorTransformed.setX(getScaleX() * vectorOriginal.getX());
      vectorTransformed.setY(getScaleY() * vectorOriginal.getY());
      vectorTransformed.setZ(getScaleZ() * vectorOriginal.getZ());
      vectorTransformed.setS(vectorOriginal.getS());

      getRotationMatrix().transform(vectorTransformed, vectorTransformed);
   }

   /**
    * Transforms the given rotation matrix by the rotation part of this rotation-scale matrix.
    * <p>
    * matrixToTransform = this.getRotationMatrix() * matrixToTransform
    * </p>
    *
    * @param matrixToTransform the rotation matrix to transform. Modified.
    */
   default void transform(RotationMatrix matrixToTransform)
   {
      transform(matrixToTransform, matrixToTransform);
   }

   /**
    * Transforms the given rotation matrix {@code matrixOriginal} by the rotation part of this
    * rotation-scale matrix and stores the result in {@code matrixTransformed}.
    * <p>
    * matrixTransformed = this.getRotationMatrix() * matrixOriginal
    * </p>
    *
    * @param matrixOriginal    the rotation matrix to transform. Not modified.
    * @param matrixTransformed the rotation matrix in which the result is stored. Modified.
    */
   default void transform(RotationMatrixReadOnly matrixOriginal, RotationMatrix matrixTransformed)
   {
      getRotationMatrix().transform(matrixOriginal, matrixTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(Matrix3DReadOnly matrixOriginal, Matrix3DBasics matrixTransformed)
   {
      matrixTransformed.set(matrixOriginal);
      // Equivalent to: M = S * M
      matrixTransformed.scaleRows(getScaleX(), getScaleY(), getScaleZ());
      // Equivalent to: M = M * S^-1
      matrixTransformed.scaleColumns(1.0 / getScaleX(), 1.0 / getScaleY(), 1.0 / getScaleZ());
      getRotationMatrix().transform(matrixTransformed, matrixTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      getRotationMatrix().inverseTransform(tupleOriginal, tupleTransformed);

      tupleTransformed.setX(tupleTransformed.getX() / getScaleX());
      tupleTransformed.setY(tupleTransformed.getY() / getScaleY());
      tupleTransformed.setZ(tupleTransformed.getZ() / getScaleZ());
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      getRotationMatrix().inverseTransform(tupleOriginal, tupleTransformed, checkIfTransformInXYPlane);

      tupleTransformed.setX(tupleTransformed.getX() / getScaleX());
      tupleTransformed.setY(tupleTransformed.getY() / getScaleY());
   }

   /**
    * Performs the inverse of the transform to the given orientation by the rotation part of this
    * rotation-scale matrix.
    * <p>
    * orientationToTransform = this.getRotationMatrix()<sup>-1</sup> * orientationToTransform <br>
    * </p>
    *
    * @param orientationToTransform the orientation to transform. Modified.
    */
   default void inverseTransform(Orientation3DBasics orientationToTransform)
   {
      inverseTransform(orientationToTransform, orientationToTransform);
   }

   /**
    * Performs the inverse of the transform to the given orientation {@code orientationOriginal} and
    * stores the result into {@code orientationTransformed}.
    * <p>
    * orientationTransformed = this.getRotationMatrix()<sup>-1</sup> * orientationOriginal <br>
    * </p>
    *
    * @param orientationOriginal    the orientation to transform. Not modified.
    * @param orientationTransformed the orientation in which the result is stored. Modified.
    */
   default void inverseTransform(Orientation3DReadOnly orientationOriginal, Orientation3DBasics orientationTransformed)
   {
      getRotationMatrix().inverseTransform(orientationOriginal, orientationTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      getRotationMatrix().inverseTransform(vectorOriginal, vectorTransformed);

      vectorTransformed.setX(vectorTransformed.getX() / getScaleX());
      vectorTransformed.setY(vectorTransformed.getY() / getScaleY());
      vectorTransformed.setZ(vectorTransformed.getZ() / getScaleZ());
   }

   /**
    * Performs the inverse of the transform to the given rotation matrix by the rotation part of this
    * rotation-scale matrix.
    * <p>
    * matrixToTransform = this.getRotationMatrix()<sup>-1</sup> * matrixToTransform
    * </p>
    * <p>
    * This operation uses the property: <br>
    * R<sup>-1</sup> = R<sup>T</sup> </br>
    * of a rotation matrix preventing to actually compute the inverse of the matrix.
    * </p>
    *
    * @param matrixToTransform the rotation matrix to transform. Modified.
    */
   default void inverseTransform(RotationMatrix matrixToTransform)
   {
      inverseTransform(matrixToTransform, matrixToTransform);
   }

   /**
    * Transforms the given rotation matrix {@code matrixOriginal} by the rotation part of this
    * rotation-scale matrix and stores the result in {@code matrixTransformed}.
    * <p>
    * matrixTransformed = this.getRotationMatrix()<sup>-1</sup> * matrixOriginal
    * </p>
    * <p>
    * This operation uses the property: <br>
    * R<sup>-1</sup> = R<sup>T</sup> </br>
    * of a rotation matrix preventing to actually compute the inverse of the matrix.
    * </p>
    *
    * @param matrixOriginal    the rotation matrix to transform. Not modified.
    * @param matrixTransformed the rotation matrix in which the result is stored. Modified.
    */
   default void inverseTransform(RotationMatrixReadOnly matrixOriginal, RotationMatrix matrixTransformed)
   {
      getRotationMatrix().inverseTransform(matrixOriginal, matrixTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Matrix3DReadOnly matrixOriginal, Matrix3DBasics matrixTransformed)
   {
      getRotationMatrix().inverseTransform(matrixOriginal, matrixTransformed);
      // Equivalent to: M = S^-1 * M
      matrixTransformed.scaleRows(1.0 / getScaleX(), 1.0 / getScaleY(), 1.0 / getScaleZ());
      // Equivalent to: M = M * S
      matrixTransformed.scaleColumns(getScaleX(), getScaleY(), getScaleZ());
   }

   /**
    * Tests the rotation parts and scales of both matrices are equal to an {@code epsilon}.
    *
    * @param other   the other matrix to compare against this. Not modified.
    * @param epsilon tolerance to use when comparing each component.
    * @return {@code true} if the two matrix are equal, {@code false} otherwise.
    */
   default boolean epsilonEquals(RotationScaleMatrixReadOnly other, double epsilon)
   {
      return getRotationMatrix().epsilonEquals(other.getRotationMatrix(), epsilon) && getScale().epsilonEquals(other.getScale(), epsilon);
   }

   /**
    * Tests if {@code this} and {@code other} represent the same rotation-scale to an {@code epsilon}.
    * <p>
    * Two rotation-scale matrices are considered geometrically equal if the their respective rotation
    * matrices and scale vectors are geometrically equal.
    * </p>
    * <p>
    * Note that {@code this.geometricallyEquals(other, epsilon) == true} does not necessarily imply
    * {@code this.epsilonEquals(other, epsilon)} and vice versa.
    * </p>
    *
    * @param other   the other rotation-scale matrix to compare against this. Not modified.
    * @param epsilon the threshold used when comparing the internal rotation and scale to
    *                {@code other}'s rotation and scale.
    * @return {@code true} if the two rotation-scale matrices represent the same geometry,
    *         {@code false} otherwise.
    */
   default boolean geometricallyEquals(RotationScaleMatrixReadOnly other, double epsilon)
   {
      return getRotationMatrix().geometricallyEquals(other.getRotationMatrix(), epsilon) && getScale().geometricallyEquals(other.getScale(), epsilon);
   }
}
