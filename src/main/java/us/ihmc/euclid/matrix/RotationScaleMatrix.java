package us.ihmc.euclid.matrix;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.exceptions.NotARotationMatrixException;
import us.ihmc.euclid.exceptions.NotARotationScaleMatrixException;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.GeometricallyComparable;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.matrix.interfaces.CommonMatrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationScaleMatrixReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tools.Matrix3DFeatures;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;

/**
 * A {@code RotationScaleMatrix} is a 3-by-3 matrix that represents a 3D orientation times a
 * diagonal matrix holding on scale factors.
 * <p>
 * The application is mostly for 3D graphics where objects are scaled in their local coordinates and
 * rotated.
 * </p>
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
 * <p>
 * A best effort has been put in the interface of {@code RotationScaleMatrix} to maximize the use of
 * the inherent properties of its composition and to minimize manipulation errors resulting in an
 * improper rotation-scale matrix.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class RotationScaleMatrix implements CommonMatrix3DBasics, RotationScaleMatrixReadOnly, Settable<RotationScaleMatrix>,
      EpsilonComparable<RotationScaleMatrix>, GeometricallyComparable<RotationScaleMatrix>
{
   /** The rotation part of this rotation-scale matrix. */
   private final RotationMatrix rotationMatrix = new RotationMatrix();
   /** The scale part of this rotation-scale matrix. */
   private final Vector3D scale = new Vector3D(1.0, 1.0, 1.0);

   /**
    * Create a new rotation-scale matrix initialized to identity.
    */
   public RotationScaleMatrix()
   {
      setIdentity();
   }

   /**
    * Creates a new rotation-scale matrix that is the same as {@code other}.
    *
    * @param other the other 3D matrix to copy the values from. Not modified.
    */
   public RotationScaleMatrix(RotationScaleMatrix other)
   {
      set(other);
   }

   /**
    * Creates a new rotation-scale matrix that is the same as {@code rotationScaleMatrix}.
    *
    * @param rotationScaleMatrix the other 3D matrix to copy the values from. Not modified.
    * @throws NotARotationScaleMatrixException if the resulting matrix is not a rotation-scale matrix.
    */
   public RotationScaleMatrix(Matrix3DReadOnly rotationScaleMatrix)
   {
      set(rotationScaleMatrix);
   }

   /**
    * Creates a new rotation-scale matrix that is the same as {@code rotationScaleMatrix}.
    *
    * @param rotationScaleMatrix the other 3D matrix to copy the values from. Not modified.
    * @throws NotARotationScaleMatrixException if the resulting matrix is not a rotation-scale matrix.
    */
   public RotationScaleMatrix(DenseMatrix64F rotationScaleMatrix)
   {
      set(rotationScaleMatrix);
   }

   /**
    * Creates a new rotation-scale matrix and initializes it from the given array.
    *
    * <pre>
    *        / rotationScaleMatrixArray[0]  rotationScaleMatrixArray[1]  rotationScaleMatrixArray[2] \
    * this = | rotationScaleMatrixArray[3]  rotationScaleMatrixArray[4]  rotationScaleMatrixArray[5] |
    *        \ rotationScaleMatrixArray[6]  rotationScaleMatrixArray[7]  rotationScaleMatrixArray[8] /
    * </pre>
    *
    * @param rotationScaleMatrixArray the array containing the values for this matrix. Not modified.
    * @throws NotARotationScaleMatrixException if the resulting matrix is not a rotation-scale matrix.
    */
   public RotationScaleMatrix(double[] rotationScaleMatrixArray)
   {
      set(rotationScaleMatrixArray);
   }

   /**
    * Creates a new rotation-scale matrix with the rotation part initialized to the
    * {@code rotationMatrix} and all three scale factors initialized to {@code scale}.
    *
    * @param rotationMatrix the 3-by-3 matrix used to initialize the rotation part. Not modified.
    * @param scale          non-zero and positive scalar used to initialize the scale factors.
    * @throws NotARotationMatrixException      if the given {@code rotationMatrix} is not a rotation
    *                                          matrix.
    * @throws NotARotationScaleMatrixException if {@code scale <= 0.0}.
    */
   public RotationScaleMatrix(DenseMatrix64F rotationMatrix, double scale)
   {
      set(rotationMatrix, scale);
   }

   /**
    * Creates a new rotation-scale matrix with the rotation part initialized to the
    * {@code rotationMatrix} and all three scale factors initialized to {@code scaleX}, {@code scaleY},
    * and {@code scaleZ}.
    *
    * @param rotationMatrix the 3-by-3 matrix used to initialize the rotation part. Not modified.
    * @param scaleX         the non-zero and positive scalar used to initialize the x-axis scale
    *                       factor.
    * @param scaleY         the non-zero and positive scalar used to initialize the y-axis scale
    *                       factor.
    * @param scaleZ         the non-zero and positive scalar used to initialize the z-axis scale
    *                       factor.
    * @throws NotARotationMatrixException      if the given {@code rotationMatrix} is not a rotation
    *                                          matrix.
    * @throws NotARotationScaleMatrixException if any of the scale factors is less or equal to zero.
    */
   public RotationScaleMatrix(DenseMatrix64F rotationMatrix, double scaleX, double scaleY, double scaleZ)
   {
      set(rotationMatrix, scaleX, scaleY, scaleZ);
   }

   /**
    * Creates a new rotation-scale matrix with the rotation part initialized to the
    * {@code rotationMatrix} and all three scale factors initialized to {@code scales}.
    *
    * @param rotationMatrix the 3-by-3 matrix used to initialize the rotation part. Not modified.
    * @param scales         tuple holding on the non-zero and positive scalars used to initialize the
    *                       scale factors. Not modified.
    * @throws NotARotationMatrixException      if the given {@code rotationMatrix} is not a rotation
    *                                          matrix.
    * @throws NotARotationScaleMatrixException if any of the scale factors is less or equal to zero.
    */
   public RotationScaleMatrix(DenseMatrix64F rotationMatrix, Tuple3DReadOnly scales)
   {
      set(rotationMatrix, scales);
   }

   /**
    * Creates a new rotation-scale matrix with the rotation part initialized to the {@code orientation}
    * and all three scale factors initialized to {@code scale}.
    *
    * @param orientation the orientation used to initialize the rotation part. Not modified.
    * @param scale       non-zero and positive scalar used to initialize the scale factors.
    * @throws NotARotationScaleMatrixException if {@code scale <= 0.0}.
    */
   public RotationScaleMatrix(Orientation3DReadOnly orientation, double scale)
   {
      set(orientation, scale);
   }

   /**
    * Creates a new rotation-scale matrix with the rotation part initialized to the {@code orientation}
    * and all three scale factors initialized to {@code scaleX}, {@code scaleY}, and {@code scaleZ}.
    *
    * @param orientation the orientation used to initialize the rotation part. Not modified.
    * @param scaleX      the non-zero and positive scalar used to initialize the x-axis scale factor.
    * @param scaleY      the non-zero and positive scalar used to initialize the y-axis scale factor.
    * @param scaleZ      the non-zero and positive scalar used to initialize the z-axis scale factor.
    * @throws NotARotationScaleMatrixException if any of the scale factors is less or equal to zero.
    */
   public RotationScaleMatrix(Orientation3DReadOnly orientation, double scaleX, double scaleY, double scaleZ)
   {
      set(orientation, scaleX, scaleY, scaleZ);
   }

   /**
    * Creates a new rotation-scale matrix with the rotation part initialized to the {@code orientation}
    * and all three scale factors initialized to {@code scales}.
    *
    * @param orientation the orientation used to initialize the rotation part. Not modified.
    * @param scales      tuple holding on the non-zero and positive scalars used to initialize the
    *                    scale factors. Not modified.
    * @throws NotARotationScaleMatrixException if any of the scale factors is less or equal to zero.
    */
   public RotationScaleMatrix(Orientation3DReadOnly orientation, Tuple3DReadOnly scales)
   {
      set(orientation, scales);
   }

   /**
    * Creates a new rotation-scale matrix with the rotation part initialized to the
    * {@code rotationMatrix} and all three scale factors initialized to {@code scale}.
    *
    * @param rotationMatrix the rotation matrix used to initialize the rotation part. Not modified.
    * @param scale          non-zero and positive scalar used to initialize the scale factors.
    * @throws NotARotationScaleMatrixException if {@code scale <= 0.0}.
    */
   public RotationScaleMatrix(RotationMatrixReadOnly rotationMatrix, double scale)
   {
      set(rotationMatrix, scale);
   }

   /**
    * Creates a new rotation-scale matrix with the rotation part initialized to the
    * {@code rotationMatrix} and all three scale factors initialized to {@code scaleX}, {@code scaleY},
    * and {@code scaleZ}.
    *
    * @param rotationMatrix the rotation matrix used to initialize the rotation part. Not modified.
    * @param scaleX         the non-zero and positive scalar used to initialize the x-axis scale
    *                       factor.
    * @param scaleY         the non-zero and positive scalar used to initialize the y-axis scale
    *                       factor.
    * @param scaleZ         the non-zero and positive scalar used to initialize the z-axis scale
    *                       factor.
    * @throws NotARotationScaleMatrixException if any of the scale factors is less or equal to zero.
    */
   public RotationScaleMatrix(RotationMatrixReadOnly rotationMatrix, double scaleX, double scaleY, double scaleZ)
   {
      set(rotationMatrix, scaleX, scaleY, scaleZ);
   }

   /**
    * Creates a new rotation-scale matrix with the rotation part initialized to the
    * {@code rotationMatrix} and all three scale factors initialized to {@code scales}.
    *
    * @param rotationMatrix the rotation matrix used to initialize the rotation part. Not modified.
    * @param scales         tuple holding on the non-zero and positive scalars used to initialize the
    *                       scale factors. Not modified.
    * @throws NotARotationScaleMatrixException if any of the scale factors is less or equal to zero.
    */
   public RotationScaleMatrix(RotationMatrixReadOnly rotationMatrix, Tuple3DReadOnly scales)
   {
      set(rotationMatrix, scales);
   }

   /**
    * Asserts that this rotation-scale matrix is proper.
    *
    * @throws NotARotationScaleMatrixException if the rotation part is not a rotation matrix.
    * @throws NotARotationScaleMatrixException if any of the scale factors is less or equal to zero.
    */
   public void checkIfRotationScaleMatrixProper()
   {
      checkIfRotationMatrixProper();
      checkIfScalesProper();
   }

   /**
    * Asserts that the rotation part of this rotation-scale matrix is proper.
    *
    * @throws NotARotationScaleMatrixException if the rotation part is not a rotation matrix.
    */
   public void checkIfRotationMatrixProper()
   {
      if (!rotationMatrix.isRotationMatrix())
         throw new NotARotationScaleMatrixException(this);
   }

   /**
    * Asserts that the scale part of this rotation-scale matrix is proper.
    *
    * @throws NotARotationScaleMatrixException if any of the scale factors is less or equal to zero.
    */
   public void checkIfScalesProper()
   {
      checkIfScalesProper(scale.getX(), scale.getY(), scale.getZ());
   }

   /**
    * Orthonormalization of the rotation part of this rotation-scale matrix using the
    * <a href="https://en.wikipedia.org/wiki/Gram%E2%80%93Schmidt_process"> Gram-Schmidt method</a>.
    *
    * @throws NotARotationMatrixException if the orthonormalization failed.
    */
   public void normalizeRotationMatrix()
   {
      rotationMatrix.normalize();
   }

   /**
    * Resets all the scale factors to 1.0.
    */
   public void resetScale()
   {
      scale.set(1.0, 1.0, 1.0);
   }

   /**
    * Sets this rotation-scale matrix to identity representing a 'zero' rotation without scale.
    */
   @Override
   public void setToZero()
   {
      setIdentity();
   }

   /** {@inheritDoc} */
   @Override
   public void setToNaN()
   {
      rotationMatrix.setToNaN();
      scale.setToNaN();
   }

   @Override
   public void setIdentity()
   {
      rotationMatrix.setIdentity();
      resetScale();
   }

   /**
    * Sets the rotation part to represent a 'zero' rotation.
    */
   public void setRotationToZero()
   {
      rotationMatrix.setToZero();
   }

   /** {@inheritDoc} */
   @Override
   public boolean containsNaN()
   {
      return rotationMatrix.containsNaN() || scale.containsNaN();
   }

   /**
    * Sets this rotation-scale matrix to equal the given one {@code other}.
    *
    * @param other the other rotation-scale matrix to copy the values from. Not modified.
    */
   @Override
   public void set(RotationScaleMatrix other)
   {
      setRotation(other.rotationMatrix);
      setScale(other.scale);
   }

   /**
    * Sets this rotation-scale matrix to equal the given one {@code other}.
    *
    * @param other the other rotation-scale matrix to copy the values from. Not modified.
    */
   public void set(RotationScaleMatrixReadOnly other)
   {
      setRotation(other.getRotationMatrix());
      setScale(other.getScale());
   }

   /**
    * Sets the rotation part to the given rotation matrix and resets the scales.
    *
    * @param rotationMatrix the other rotation matrix to copy the values from. Not modified.
    */
   public void set(RotationMatrixReadOnly rotationMatrix)
   {
      setRotation(rotationMatrix);
      resetScale();
   }

   /**
    * {@inheritDoc}
    *
    * @throws NotARotationScaleMatrixException if the resulting matrix is not a rotation-scale matrix.
    */
   @Override
   public void set(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
   {
      if (Matrix3DFeatures.determinant(m00, m01, m02, m10, m11, m12, m20, m21, m22) <= 0.0)
         throw new NotARotationScaleMatrixException(m00, m01, m02, m10, m11, m12, m20, m21, m22);

      scale.setX(EuclidCoreTools.squareRoot(m00 * m00 + m10 * m10 + m20 * m20));
      scale.setY(EuclidCoreTools.squareRoot(m01 * m01 + m11 * m11 + m21 * m21));
      scale.setZ(EuclidCoreTools.squareRoot(m02 * m02 + m12 * m12 + m22 * m22));

      if (scale.getX() == 0.0 || scale.getY() == 0.0 || scale.getZ() == 0.0)
         throw new NotARotationScaleMatrixException(m00, m01, m02, m10, m11, m12, m20, m21, m22);

      double rot00 = m00 / scale.getX();
      double rot01 = m01 / scale.getY();
      double rot02 = m02 / scale.getZ();
      double rot10 = m10 / scale.getX();
      double rot11 = m11 / scale.getY();
      double rot12 = m12 / scale.getZ();
      double rot20 = m20 / scale.getX();
      double rot21 = m21 / scale.getY();
      double rot22 = m22 / scale.getZ();

      if (!Matrix3DFeatures.isRotationMatrix(rot00, rot01, rot02, rot10, rot11, rot12, rot20, rot21, rot22))
         throw new NotARotationScaleMatrixException(m00, m01, m02, m10, m11, m12, m20, m21, m22);

      rotationMatrix.setUnsafe(rot00, rot01, rot02, rot10, rot11, rot12, rot20, rot21, rot22);
   }

   /**
    * Sets the rotation part to the {@code rotationMatrix} and all three scale factors to
    * {@code scale}.
    *
    * @param rotationMatrix the 3-by-3 matrix used to set the rotation part to. Not modified.
    * @param scale          non-zero and positive scalar used to initialize the scale factors.
    * @throws NotARotationMatrixException      if the given {@code rotationMatrix} is not a rotation
    *                                          matrix.
    * @throws NotARotationScaleMatrixException if {@code scale <= 0.0}.
    */
   public void set(DenseMatrix64F rotationMatrix, double scale)
   {
      set(rotationMatrix, scale, scale, scale);
   }

   /**
    * Sets the rotation part to the {@code rotationMatrix} and all three scale factors to
    * {@code scaleX}, {@code scaleY}, and {@code scaleZ}.
    *
    * @param rotationMatrix the 3-by-3 matrix used to set the rotation part to. Not modified.
    * @param scaleX         the non-zero and positive scalar used to set the x-axis scale factor to.
    * @param scaleY         the non-zero and positive scalar used to set the y-axis scale factor to.
    * @param scaleZ         the non-zero and positive scalar used to set the z-axis scale factor to.
    * @throws NotARotationMatrixException      if the given {@code rotationMatrix} is not a rotation
    *                                          matrix.
    * @throws NotARotationScaleMatrixException if any of the scale factors is less or equal to zero.
    */
   public void set(DenseMatrix64F rotationMatrix, double scaleX, double scaleY, double scaleZ)
   {
      setRotation(rotationMatrix);
      setScale(scaleX, scaleY, scaleZ);
   }

   /**
    * Sets the rotation part to the {@code rotationMatrix} and all three scale factors to
    * {@code scales}.
    *
    * @param rotationMatrix the 3-by-3 matrix used to set the rotation part to. Not modified.
    * @param scales         tuple holding on the non-zero and positive scalars used to set the scale
    *                       factors to. Not modified.
    * @throws NotARotationMatrixException      if the given {@code rotationMatrix} is not a rotation
    *                                          matrix.
    * @throws NotARotationScaleMatrixException if any of the scale factors is less or equal to zero.
    */
   public void set(DenseMatrix64F rotationMatrix, Tuple3DReadOnly scales)
   {
      set(rotationMatrix, scales.getX(), scales.getY(), scales.getZ());
   }

   /**
    * Sets the rotation part to the {@code orientation} and resets the scales.
    *
    * @param orientation the orientation used to set the rotation part to. Not modified.
    */
   public void set(Orientation3DReadOnly orientation)
   {
      setRotation(orientation);
      resetScale();
   }

   /**
    * Sets the rotation part to the {@code orientation} and all three scale factors to {@code scale}.
    *
    * @param orientation the orientation used to set the rotation part to. Not modified.
    * @param scale       the non-zero and positive scalar used to set the scale factors to.
    * @throws NotARotationScaleMatrixException if {@code scale <= 0.0}.
    */
   public void set(Orientation3DReadOnly orientation, double scale)
   {
      set(orientation, scale, scale, scale);
   }

   /**
    * Sets the rotation part to the {@code orientation} and all three scale factors to {@code scaleX},
    * {@code scaleY}, and {@code scaleZ}.
    *
    * @param orientation the orientation used to set the rotation part to. Not modified.
    * @param scaleX      the non-zero and positive scalar used to set the x-axis scale factor to.
    * @param scaleY      the non-zero and positive scalar used to set the y-axis scale factor to.
    * @param scaleZ      the non-zero and positive scalar used to set the z-axis scale factor to.
    * @throws NotARotationScaleMatrixException if any of the scale factors is less or equal to zero.
    */
   public void set(Orientation3DReadOnly orientation, double scaleX, double scaleY, double scaleZ)
   {
      setRotation(orientation);
      setScale(scaleX, scaleY, scaleZ);
   }

   /**
    * Sets the rotation part to the {@code orientation} and all three scale factors to {@code scales}.
    *
    * @param orientation the orientation used to set the rotation part to. Not modified.
    * @param scales      tuple holding on the non-zero and positive scalars used to set the scale
    *                    factors to. Not modified.
    * @throws NotARotationScaleMatrixException if any of the scale factors is less or equal to zero.
    */
   public void set(Orientation3DReadOnly orientation, Tuple3DReadOnly scales)
   {
      set(orientation, scales.getX(), scales.getY(), scales.getZ());
   }

   /**
    * Sets the rotation part to the {@code rotationMatrix} and all three scale factors to
    * {@code scale}.
    *
    * @param rotationMatrix the matrix used to set the rotation part to. Not modified.
    * @param scale          the non-zero and positive scalar used to set the scale factors to.
    * @throws NotARotationMatrixException      if the given {@code rotationMatrix} is not a rotation
    *                                          matrix.
    * @throws NotARotationScaleMatrixException if {@code scale <= 0.0}.
    */
   public void set(Matrix3DReadOnly rotationMatrix, double scale)
   {
      set(rotationMatrix, scale, scale, scale);
   }

   /**
    * Sets the rotation part to the {@code rotationMatrix} and all three scale factors to
    * {@code scaleX}, {@code scaleY}, and {@code scaleZ}.
    *
    * @param rotationMatrix the rotation matrix used to set the rotation part to. Not modified.
    * @param scaleX         the non-zero and positive scalar used to set the x-axis scale factor to.
    * @param scaleY         the non-zero and positive scalar used to set the y-axis scale factor to.
    * @param scaleZ         the non-zero and positive scalar used to set the z-axis scale factor to.
    * @throws NotARotationMatrixException      if the given {@code rotationMatrix} is not a rotation
    *                                          matrix.
    * @throws NotARotationScaleMatrixException if any of the scale factors is less or equal to zero.
    */
   public void set(Matrix3DReadOnly rotationMatrix, double scaleX, double scaleY, double scaleZ)
   {
      setRotation(rotationMatrix);
      setScale(scaleX, scaleY, scaleZ);
   }

   /**
    * Sets the rotation part to the {@code rotationMatrix} and all three scale factors to
    * {@code scales}.
    *
    * @param rotationMatrix the rotation matrix used to set the rotation part to. Not modified.
    * @param scales         tuple holding on the non-zero and positive scalars used to set the scale
    *                       factors to. Not modified.
    * @throws NotARotationMatrixException      if the given {@code rotationMatrix} is not a rotation
    *                                          matrix.
    * @throws NotARotationScaleMatrixException if any of the scale factors is less or equal to zero.
    */
   public void set(Matrix3DReadOnly rotationMatrix, Tuple3DReadOnly scales)
   {
      set(rotationMatrix, scales.getX(), scales.getY(), scales.getZ());
   }

   /**
    * Sets the rotation part to the {@code rotationMatrix} and all three scale factors to
    * {@code scale}.
    *
    * @param rotationMatrix the matrix used to set the rotation part to. Not modified.
    * @param scale          the non-zero and positive scalar used to set the scale factors to.
    * @throws NotARotationScaleMatrixException if {@code scale <= 0.0}.
    */
   public void set(RotationMatrixReadOnly rotationMatrix, double scale)
   {
      set(rotationMatrix, scale, scale, scale);
   }

   /**
    * Sets the rotation part to the {@code rotationMatrix} and all three scale factors to
    * {@code scaleX}, {@code scaleY}, and {@code scaleZ}.
    *
    * @param rotationMatrix the rotation matrix used to set the rotation part to. Not modified.
    * @param scaleX         the non-zero and positive scalar used to set the x-axis scale factor to.
    * @param scaleY         the non-zero and positive scalar used to set the y-axis scale factor to.
    * @param scaleZ         the non-zero and positive scalar used to set the z-axis scale factor to.
    * @throws NotARotationScaleMatrixException if any of the scale factors is less or equal to zero.
    */
   public void set(RotationMatrixReadOnly rotationMatrix, double scaleX, double scaleY, double scaleZ)
   {
      setRotation(rotationMatrix);
      setScale(scaleX, scaleY, scaleZ);
   }

   /**
    * Sets the rotation part to the {@code rotationMatrix} and all three scale factors to
    * {@code scales}.
    *
    * @param rotationMatrix the rotation matrix used to set the rotation part to. Not modified.
    * @param scales         tuple holding on the non-zero and positive scalars used to set the scale
    *                       factors to. Not modified.
    * @throws NotARotationScaleMatrixException if any of the scale factors is less or equal to zero.
    */
   public void set(RotationMatrixReadOnly rotationMatrix, Tuple3DReadOnly scales)
   {
      set(rotationMatrix, scales.getX(), scales.getY(), scales.getZ());
   }

   /**
    * Sets the rotation part to {@code rotationMatrix}.
    *
    * @param rotationMatrix the matrix used to set the rotation part to. Not modified.
    * @throws NotARotationMatrixException if the given {@code rotationMatrix} is not a rotation matrix.
    */
   public void setRotation(DenseMatrix64F rotationMatrix)
   {
      this.rotationMatrix.set(rotationMatrix);
   }

   /**
    * Sets the rotation part to {@code rotationMatrixArray}.
    *
    * @param rotationMatrixArray the array containing the rotation part values. Not modified.
    * @throws NotARotationMatrixException if the given {@code rotationMatrixArray} is not a rotation
    *                                     matrix.
    */
   public void setRotation(double[] rotationMatrixArray)
   {
      rotationMatrix.set(rotationMatrixArray);
   }

   /**
    * Sets the rotation part to {@code orientation}.
    *
    * @param orientation the orientation used to set the rotation part to. Not modified.
    */
   public void setRotation(Orientation3DReadOnly orientation)
   {
      rotationMatrix.set(orientation);
   }

   /**
    * Sets the rotation part to {@code rotationMatrix}.
    *
    * @param rotationMatrix the matrix used to set the rotation part to. Not modified.
    * @throws NotARotationMatrixException if the given {@code rotationMatrix} is not a rotation matrix.
    */
   public void setRotation(Matrix3DReadOnly rotationMatrix)
   {
      this.rotationMatrix.set(rotationMatrix);
   }

   /**
    * Sets the rotation part to {@code rotationMatrix}.
    *
    * @param rotationMatrix the matrix used to set the rotation part to. Not modified.
    */
   public void setRotation(RotationMatrixReadOnly rotationMatrix)
   {
      this.rotationMatrix.set(rotationMatrix);
   }

   /**
    * Sets the rotation part to the rotation vector {@code rotationVector}.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation. A
    * rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle of the
    * same axis-angle.
    * </p>
    *
    * @param rotationVector the rotation vector used to set the rotation part to. Not modified.
    */
   public void setRotation(Vector3DReadOnly rotationVector)
   {
      rotationMatrix.setRotationVector(rotationVector);
   }

   /**
    * Sets the rotation part to represent a counter clockwise rotation around the z-axis of an angle
    * {@code yaw}.
    *
    * <pre>
    *     / cos(yaw) -sin(yaw) 0 \
    * R = | sin(yaw)  cos(yaw) 0 |
    *     \    0         0     1 /
    * </pre>
    *
    * @param yaw the angle to rotate about the z-axis.
    */
   public void setRotationYaw(double yaw)
   {
      rotationMatrix.setToYawOrientation(yaw);
   }

   /**
    * Sets the rotation part to represent a counter clockwise rotation around the y-axis of an angle
    * {@code pitch}.
    *
    * <pre>
    *        /  cos(pitch) 0 sin(pitch) \
    * this = |      0      1     0      |
    *        \ -sin(pitch) 0 cos(pitch) /
    * </pre>
    *
    * @param pitch the angle to rotate about the y-axis.
    */
   public void setRotationPitch(double pitch)
   {
      rotationMatrix.setToPitchOrientation(pitch);
   }

   /**
    * Sets the rotation part to represent a counter clockwise rotation around the x-axis of an angle
    * {@code roll}.
    *
    * <pre>
    *        / 1     0          0     \
    * this = | 0 cos(roll) -sin(roll) |
    *        \ 0 sin(roll)  cos(roll) /
    * </pre>
    *
    * @param roll the angle to rotate about the x-axis.
    */
   public void setRotationRoll(double roll)
   {
      rotationMatrix.setToRollOrientation(roll);
   }

   /**
    * Sets the rotation part to represent the same orientation as the given yaw-pitch-roll angles
    * {@code yawPitchRoll}.
    *
    * <pre>
    *     / cos(yaw) -sin(yaw) 0 \   /  cos(pitch) 0 sin(pitch) \   / 1     0          0     \
    * R = | sin(yaw)  cos(yaw) 0 | * |      0      1     0      | * | 0 cos(roll) -sin(roll) |
    *     \    0         0     1 /   \ -sin(pitch) 0 cos(pitch) /   \ 0 sin(roll)  cos(roll) /
    * </pre>
    *
    * @param yawPitchRoll the yaw-pitch-roll Euler angles to copy the orientation from. Not modified.
    * @deprecated Use {@link #setRotation(Orientation3DReadOnly)} with {@link YawPitchRoll}.
    */
   @Deprecated
   public void setRotationYawPitchRoll(double[] yawPitchRoll)
   {
      setRotationYawPitchRoll(yawPitchRoll[0], yawPitchRoll[1], yawPitchRoll[2]);
   }

   /**
    * Sets the rotation part to represent the same orientation as the given yaw-pitch-roll angles
    * {@code yaw}, {@code pitch}, and {@code roll}.
    *
    * <pre>
    *     / cos(yaw) -sin(yaw) 0 \   /  cos(pitch) 0 sin(pitch) \   / 1     0          0     \
    * R = | sin(yaw)  cos(yaw) 0 | * |      0      1     0      | * | 0 cos(roll) -sin(roll) |
    *     \    0         0     1 /   \ -sin(pitch) 0 cos(pitch) /   \ 0 sin(roll)  cos(roll) /
    * </pre>
    *
    * @param yaw   the angle to rotate about the z-axis.
    * @param pitch the angle to rotate about the y-axis.
    * @param roll  the angle to rotate about the x-axis.
    */
   public void setRotationYawPitchRoll(double yaw, double pitch, double roll)
   {
      rotationMatrix.setYawPitchRoll(yaw, pitch, roll);
   }

   /**
    * Sets the rotation part to represent the same orientation as the given Euler angles
    * {@code eulerAngles}.
    *
    * <pre>
    *     / cos(eulerAngles.z) -sin(eulerAngles.z) 0 \   /  cos(eulerAngles.y) 0 sin(eulerAngles.y) \   / 1         0                   0          \
    * R = | sin(eulerAngles.z)  cos(eulerAngles.z) 0 | * |          0          1         0          | * | 0 cos(eulerAngles.x) -sin(eulerAngles.x) |
    *     \         0                   0          1 /   \ -sin(eulerAngles.y) 0 cos(eulerAngles.y) /   \ 0 sin(eulerAngles.x)  cos(eulerAngles.x) /
    * </pre>
    * <p>
    * This is equivalent to
    * {@code this.setRotationYawPitchRoll(eulerAngles.getZ(), eulerAngles.getY(), eulerAngles.getX())}.
    * </p>
    *
    * @param eulerAngles the Euler angles to copy the orientation from. Not modified.
    */
   public void setRotationEuler(Vector3DReadOnly eulerAngles)
   {
      rotationMatrix.setEuler(eulerAngles);
   }

   /**
    * Sets the rotation part to represent the same orientation as the given Euler angles {@code rotX},
    * {@code rotY}, and {@code rotZ}.
    *
    * <pre>
    *        / cos(rotZ) -sin(rotZ) 0 \   /  cos(rotY) 0 sin(rotY) \   / 1     0          0     \
    * this = | sin(rotZ)  cos(rotZ) 0 | * |      0     1     0     | * | 0 cos(rotX) -sin(rotX) |
    *        \     0          0     1 /   \ -sin(rotY) 0 cos(rotY) /   \ 0 sin(rotX)  cos(rotX) /
    * </pre>
    * <p>
    * This is equivalent to {@code this.setRotationYawPitchRoll(rotZ, rotY, rotX)}.
    * </p>
    *
    * @param rotX the angle to rotate about the x-axis.
    * @param rotY the angle to rotate about the y-axis.
    * @param rotZ the angle to rotate about the z-axis.
    */
   public void setRotationEuler(double rotX, double rotY, double rotZ)
   {
      rotationMatrix.setEuler(rotX, rotY, rotZ);
   }

   /**
    * Sets all the scale factors to {@code scale}.
    *
    * @param scale the non-zero and positive scalar used to set the scale factors to.
    * @throws NotARotationScaleMatrixException if {@code scale <= 0.0}.
    */
   public void setScale(double scale)
   {
      setScale(scale, scale, scale);
   }

   /**
    * Sets all the scale factors to {@code scale}.
    *
    * @param scaleX the non-zero and positive scalar used to set the x-axis scale factor to.
    * @param scaleY the non-zero and positive scalar used to set the y-axis scale factor to.
    * @param scaleZ the non-zero and positive scalar used to set the z-axis scale factor to.
    * @throws NotARotationScaleMatrixException if any of the scale factors is less or equal to zero.
    */
   public void setScale(double scaleX, double scaleY, double scaleZ)
   {
      checkIfScalesProper(scaleX, scaleY, scaleZ);

      scale.set(scaleX, scaleY, scaleZ);
   }

   /**
    * Sets the scale factors to {@code scales}.
    *
    * @param scales tuple holding on the non-zero and positive scalars used to set the scale factors
    *               to. Not modified.
    * @throws NotARotationScaleMatrixException if any of the scale factors is less or equal to zero.
    */
   public void setScale(Tuple3DReadOnly scales)
   {
      setScale(scales.getX(), scales.getY(), scales.getZ());
   }

   private void checkIfScalesProper(double scaleX, double scaleY, double scaleZ)
   {
      if (scaleX < 0.0 || scaleY < 0.0 || scaleZ < 0.0)
         throw new NotARotationScaleMatrixException("Mirroring is not handled, scale values: " + scaleX + ", " + scaleY + ", " + scaleZ + ".");
   }

   /**
    * Resets the scale factors and sets the rotation part to represent a counter clockwise rotation
    * around the z-axis of an angle {@code yaw}.
    *
    * <pre>
    *        / cos(yaw) -sin(yaw) 0 \
    * this = | sin(yaw)  cos(yaw) 0 |
    *        \    0         0     1 /
    * </pre>
    *
    * @param yaw the angle to rotate about the z-axis.
    */
   public void setToYawMatrix(double yaw)
   {
      setRotationYaw(yaw);
      resetScale();
   }

   /**
    * Resets the scale factors and sets the rotation part to represent a counter clockwise rotation
    * around the y-axis of an angle {@code pitch}.
    *
    * <pre>
    *        /  cos(pitch) 0 sin(pitch) \
    * this = |      0      1     0      |
    *        \ -sin(pitch) 0 cos(pitch) /
    * </pre>
    *
    * @param pitch the angle to rotate about the y-axis.
    */
   public void setToPitchMatrix(double pitch)
   {
      setRotationPitch(pitch);
      resetScale();
   }

   /**
    * Resets the scale factors and sets the rotation part to represent a counter clockwise rotation
    * around the x-axis of an angle {@code roll}.
    *
    * <pre>
    *        / 1     0          0     \
    * this = | 0 cos(roll) -sin(roll) |
    *        \ 0 sin(roll)  cos(roll) /
    * </pre>
    *
    * @param roll the angle to rotate about the x-axis.
    */
   public void setToRollMatrix(double roll)
   {
      setRotationRoll(roll);
      resetScale();
   }

   /**
    * Resets the scale factors and sets the rotation part to represent the same orientation as the
    * given yaw-pitch-roll angles {@code yawPitchRoll}.
    *
    * <pre>
    *        / cos(yaw) -sin(yaw) 0 \   /  cos(pitch) 0 sin(pitch) \   / 1     0          0     \
    * this = | sin(yaw)  cos(yaw) 0 | * |      0      1     0      | * | 0 cos(roll) -sin(roll) |
    *        \    0         0     1 /   \ -sin(pitch) 0 cos(pitch) /   \ 0 sin(roll)  cos(roll) /
    * </pre>
    *
    * @param yawPitchRoll the yaw-pitch-roll Euler angles to copy the orientation from. Not modified.
    * @deprecated Use {@link #set(Orientation3DReadOnly, double)} with {@link YawPitchRoll}.
    */
   @Deprecated
   public void setYawPitchRoll(double[] yawPitchRoll)
   {
      setYawPitchRoll(yawPitchRoll[0], yawPitchRoll[1], yawPitchRoll[2]);
   }

   /**
    * Resets the scale factors and sets the rotation part to represent the same orientation as the
    * given yaw-pitch-roll angles {@code yaw}, {@code pitch}, and {@code roll}.
    *
    * <pre>
    *        / cos(yaw) -sin(yaw) 0 \   /  cos(pitch) 0 sin(pitch) \   / 1     0          0     \
    * this = | sin(yaw)  cos(yaw) 0 | * |      0      1     0      | * | 0 cos(roll) -sin(roll) |
    *        \    0         0     1 /   \ -sin(pitch) 0 cos(pitch) /   \ 0 sin(roll)  cos(roll) /
    * </pre>
    *
    * @param yaw   the angle to rotate about the z-axis.
    * @param pitch the angle to rotate about the y-axis.
    * @param roll  the angle to rotate about the x-axis.
    */
   public void setYawPitchRoll(double yaw, double pitch, double roll)
   {
      setRotationYawPitchRoll(yaw, pitch, roll);
      resetScale();
   }

   /**
    * Resets the scale factors and sets the rotation part to represent the same orientation as the
    * given Euler angles {@code eulerAngles}.
    *
    * <pre>
    *        / cos(eulerAngles.z) -sin(eulerAngles.z) 0 \   /  cos(eulerAngles.y) 0 sin(eulerAngles.y) \   / 1         0                   0          \
    * this = | sin(eulerAngles.z)  cos(eulerAngles.z) 0 | * |          0          1         0          | * | 0 cos(eulerAngles.x) -sin(eulerAngles.x) |
    *        \         0                   0          1 /   \ -sin(eulerAngles.y) 0 cos(eulerAngles.y) /   \ 0 sin(eulerAngles.x)  cos(eulerAngles.x) /
    * </pre>
    * <p>
    * This is equivalent to
    * {@code this.setYawPitchRoll(eulerAngles.getZ(), eulerAngles.getY(), eulerAngles.getX())}.
    * </p>
    *
    * @param eulerAngles the Euler angles to copy the orientation from. Not modified.
    */
   public void setEuler(Vector3DReadOnly eulerAngles)
   {
      setRotationEuler(eulerAngles);
      resetScale();
   }

   /**
    * Resets the scale factors and sets the rotation part to represent the same orientation as the
    * given Euler angles {@code rotX}, {@code rotY}, and {@code rotZ}.
    *
    * <pre>
    *        / cos(rotZ) -sin(rotZ) 0 \   /  cos(rotY) 0 sin(rotY) \   / 1     0          0     \
    * this = | sin(rotZ)  cos(rotZ) 0 | * |      0     1     0     | * | 0 cos(rotX) -sin(rotX) |
    *        \     0          0     1 /   \ -sin(rotY) 0 cos(rotY) /   \ 0 sin(rotX)  cos(rotX) /
    * </pre>
    * <p>
    * This is equivalent to {@code this.setYawPitchRoll(rotZ, rotY, rotX)}.
    * </p>
    *
    * @param rotX the angle to rotate about the x-axis.
    * @param rotY the angle to rotate about the y-axis.
    * @param rotZ the angle to rotate about the z-axis.
    */
   public void setEuler(double rotX, double rotY, double rotZ)
   {
      setRotationEuler(rotX, rotY, rotZ);
      resetScale();
   }

   /**
    * Appends the given orientation to the rotation part of {@code this}.
    *
    * @param orientation the orientation to append to {@code this}. Not modified.
    */
   public void append(Orientation3DReadOnly orientation)
   {
      rotationMatrix.append(orientation);
   }

   /**
    * Inverts the rotation part of {@code this} and appends the given orientation.
    *
    * @param orientation the orientation to append to {@code this}. Not modified.
    */
   public void appendInvertThis(Orientation3DReadOnly orientation)
   {
      rotationMatrix.appendInvertThis(orientation);
   }

   /**
    * Appends the inverse of the given orientation to the rotation part of {@code this}.
    *
    * @param orientation the orientation to append to {@code this}. Not modified.
    */
   public void appendInvertOther(Orientation3DReadOnly orientation)
   {
      rotationMatrix.appendInvertOther(orientation);
   }

   /**
    * Appends a rotation about the z-axis to the rotation part of this rotation-scale matrix.
    *
    * <pre>
    *         / cos(yaw) -sin(yaw) 0 \
    * R = R * | sin(yaw)  cos(yaw) 0 |
    *         \    0         0     1 /
    * </pre>
    * <p>
    * This method does not affect the scale part of this rotation-scale matrix.
    * </p>
    *
    * @param yaw the angle to rotate about the z-axis.
    */
   public void appendYawRotation(double yaw)
   {
      rotationMatrix.appendYawRotation(yaw);
   }

   /**
    * Appends a rotation about the y-axis to the rotation part of this rotation-scale matrix.
    *
    * <pre>
    *         /  cos(pitch) 0 sin(pitch) \
    * R = R * |      0      1     0      |
    *         \ -sin(pitch) 0 cos(pitch) /
    * </pre>
    * <p>
    * This method does not affect the scale part of this rotation-scale matrix.
    * </p>
    *
    * @param pitch the angle to rotate about the y-axis.
    */
   public void appendPitchRotation(double pitch)
   {
      rotationMatrix.appendPitchRotation(pitch);
   }

   /**
    * Appends a rotation about the x-axis to the rotation part of this rotation-scale matrix.
    *
    * <pre>
    *         / 1     0          0     \
    * R = R * | 0 cos(roll) -sin(roll) |
    *         \ 0 sin(roll)  cos(roll) /
    * </pre>
    * <p>
    * This method does not affect the scale part of this rotation-scale matrix.
    * </p>
    *
    * @param roll the angle to rotate about the x-axis.
    */
   public void appendRollRotation(double roll)
   {
      rotationMatrix.appendRollRotation(roll);
   }

   /**
    * Prepends the given orientation to the rotation part of {@code this}.
    *
    * @param orientation the orientation to prepend to {@code this}. Not modified.
    */
   public void prepend(Orientation3DReadOnly orientation)
   {
      rotationMatrix.prepend(orientation);
   }

   /**
    * Inverts the rotation part of {@code this} and prepends the given orientation.
    *
    * @param orientation the orientation to prepend to {@code this}. Not modified.
    */
   public void prependInvertThis(Orientation3DReadOnly orientation)
   {
      rotationMatrix.prependInvertThis(orientation);
   }

   /**
    * Prepends the inverse of the given orientation to the rotation part of {@code this}.
    *
    * @param orientation the orientation to prepend to {@code this}. Not modified.
    */
   public void prependInvertOther(Orientation3DReadOnly orientation)
   {
      rotationMatrix.prependInvertOther(orientation);
   }

   /**
    * Prepend a rotation about the z-axis to the rotation part of this rotation-scale matrix.
    *
    * <pre>
    *     / cos(yaw) -sin(yaw) 0 \
    * R = | sin(yaw)  cos(yaw) 0 | * R
    *     \    0         0     1 /
    * </pre>
    * <p>
    * This method does not affect the scale part of this rotation-scale matrix.
    * </p>
    *
    * @param yaw the angle to rotate about the z-axis.
    */
   public void prependYawRotation(double yaw)
   {
      rotationMatrix.prependYawRotation(yaw);
   }

   /**
    * Prepend a rotation about the y-axis to the rotation part of this rotation-scale matrix.
    *
    * <pre>
    *     /  cos(pitch) 0 sin(pitch) \
    * R = |      0      1     0      | * R
    *     \ -sin(pitch) 0 cos(pitch) /
    * </pre>
    * <p>
    * This method does not affect the scale part of this rotation-scale matrix.
    * </p>
    *
    * @param pitch the angle to rotate about the y-axis.
    */
   public void prependPitchRotation(double pitch)
   {
      rotationMatrix.prependPitchRotation(pitch);
   }

   /**
    * Prepend a rotation about the x-axis to the rotation part of this rotation-scale matrix.
    *
    * <pre>
    *     / 1     0          0     \
    * R = | 0 cos(roll) -sin(roll) | * R
    *     \ 0 sin(roll)  cos(roll) /
    * </pre>
    * <p>
    * This method does not affect the scale part of this rotation-scale matrix.
    * </p>
    *
    * @param roll the angle to rotate about the x-axis.
    */
   public void prependRollRotation(double roll)
   {
      rotationMatrix.prependRollRotation(roll);
   }

   /**
    * Returns the reference to the rotation matrix used to compose this rotation-scale matrix.
    *
    * @return the reference to the rotation matrix.
    */
   @Override
   public RotationMatrix getRotationMatrix()
   {
      return rotationMatrix;
   }

   /** {@inheritDoc} */
   @Override
   public Vector3DReadOnly getScale()
   {
      return scale;
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(RotationScaleMatrix)}, it returns {@code false} otherwise or if the {@code object}
    * is {@code null}.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof Matrix3DReadOnly)
         return equals((Matrix3DReadOnly) object);
      else
         return false;
   }

   /**
    * Tests if the rotation parts and scales of both matrices are exactly equal.
    * <p>
    * The method returns {@code false} if the given matrix is {@code null}.
    * </p>
    *
    * @param other the other matrix to compare against this. Not modified.
    * @return {@code true} if the two matrices are exactly equal, {@code false} otherwise.
    */
   public boolean equals(RotationScaleMatrix other)
   {
      if (other == null)
         return false;
      else
         return rotationMatrix.equals(other.rotationMatrix) && scale.equals(other.scale);
   }

   /**
    * Provides a {@code String} representation of this matrix as follows: <br>
    * m00, m01, m02 <br>
    * m10, m11, m12 <br>
    * m20, m21, m22
    *
    * @return the {@code String} representing this matrix.
    */
   @Override
   public String toString()
   {
      return EuclidCoreIOTools.getMatrixString(this);
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this matrix.
    *
    * @return the hash code value for this matrix.
    */
   @Override
   public int hashCode()
   {
      long bits = EuclidHashCodeTools.combineHashCode(rotationMatrix.hashCode(), scale.hashCode());
      return EuclidHashCodeTools.toIntHashCode(bits);
   }

   /**
    * Tests the rotation parts and scales of both matrices are equal to an {@code epsilon}.
    *
    * @param other   the other matrix to compare against this. Not modified.
    * @param epsilon tolerance to use when comparing each component.
    * @return {@code true} if the two matrix are equal, {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(RotationScaleMatrix other, double epsilon)
   {
      return RotationScaleMatrixReadOnly.super.epsilonEquals(other, epsilon);
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
   @Override
   public boolean geometricallyEquals(RotationScaleMatrix other, double epsilon)
   {
      return RotationScaleMatrixReadOnly.super.geometricallyEquals(other, epsilon);
   }
}