package us.ihmc.euclid.transform;

import org.ejml.data.DMatrix;

import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.GeometricallyComparable;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.matrix.LinearTransform3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tools.Matrix3DTools;
import us.ihmc.euclid.transform.interfaces.AffineTransformBasics;
import us.ihmc.euclid.transform.interfaces.AffineTransformReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public class AffineTransform
      implements AffineTransformBasics, EpsilonComparable<AffineTransform>, GeometricallyComparable<AffineTransform>, Settable<AffineTransform>
{
   /** The rotation plus scaling part of this transform. */
   private final LinearTransform3D linearTransform = new LinearTransform3D();
   /** The translation part of this transform. */
   private final Vector3D translationVector = new Vector3D();

   /**
    * Creates a new affine transform set to identity.
    * <p>
    * When set to identity, this transform has no effect when transforming a geometry object.
    * </p>
    */
   public AffineTransform()
   {
   }

   /**
    * Creates a new affine transform and sets it to {@code other}.
    *
    * @param other the other affine transform to copy. Not modified.
    */
   public AffineTransform(AffineTransformReadOnly other)
   {
      set(other);
   }

   /**
    * Creates a new affine transform and sets it to {@code rigidBodyTransform}.
    * <p>
    * This affine transform has no scaling (1.0, 1.0, 1.0).
    * </p>
    *
    * @param rigidBodyTransform the rigid-body transform to copy. Not modified.
    */
   public AffineTransform(RigidBodyTransformReadOnly rigidBodyTransform)
   {
      set(rigidBodyTransform);
   }

   /**
    * Creates a new affine transform and initializes it from the given rotation-scale matrix and the
    * given translation.
    *
    * @param linearTransform the rotation-scale matrix to copy. Not modified.
    * @param translation     the translation to copy. Not modified.
    */
   public AffineTransform(Matrix3DReadOnly linearTransform, Tuple3DReadOnly translation)
   {
      set(linearTransform, translation);
   }

   public AffineTransform(DMatrix linearTransform, Tuple3DReadOnly translation)
   {
      set(linearTransform, translation);
   }

   public AffineTransform(RotationMatrix rotationMatrix, Tuple3DReadOnly translation)
   {
      set(rotationMatrix, translation);
   }

   public AffineTransform(Orientation3DReadOnly orientation, Tuple3DReadOnly translation)
   {
      set(orientation, translation);
   }

   /**
    * Sets this affine transform to the {@code other}.
    *
    * @param other the other affine transform to copy the values from. Not modified.
    */
   @Override
   public void set(AffineTransform other)
   {
      set((AffineTransformReadOnly) other);
   }

   public QuaternionReadOnly getRotationView()
   {
      return linearTransform.getAsQuaternion();
   }

   /**
    * Gets the read-only reference to the rotation-scale part of this transform.
    *
    * @return the rotation-scale part of this transform.
    */
   @Override
   public LinearTransform3D getLinearTransform()
   {
      return linearTransform;
   }

   /**
    * Gets the read-only reference of the translation part of this affine transform.
    *
    * @return the translation part of this transform.
    */
   @Override
   public Vector3DBasics getTranslation()
   {
      return translationVector;
   }

   /**
    * Retrieves and returns a coefficient of this transform given its row and column indices.
    *
    * @param row    the row of the coefficient to return.
    * @param column the column of the coefficient to return.
    * @return the coefficient's value.
    * @throws ArrayIndexOutOfBoundsException if either {@code row} &notin; [0, 3] or {@code column}
    *                                        &notin; [0, 3].
    */
   public double getElement(int row, int column)
   {
      if (row < 3)
      {
         if (column < 3)
         {
            return linearTransform.getElement(row, column);
         }
         else if (column < 4)
         {
            return translationVector.getElement(row);
         }
         else
         {
            throw Matrix3DTools.columnOutOfBoundsException(3, column);
         }
      }
      else if (row < 4)
      {
         if (column < 3)
         {
            return 0.0;
         }
         else if (column < 4)
         {
            return 1.0;
         }
         else
         {
            throw Matrix3DTools.columnOutOfBoundsException(3, column);
         }
      }
      else
      {
         throw Matrix3DTools.rowOutOfBoundsException(3, row);
      }
   }

   /**
    * Gets the 1st row 1st column coefficient of this transform.
    *
    * @return the 1st row 1st column coefficient.
    */
   public double getM00()
   {
      return linearTransform.getM00();
   }

   /**
    * Gets the 1st row 2nd column coefficient of this transform.
    *
    * @return the 1st row 2nd column coefficient.
    */
   public double getM01()
   {
      return linearTransform.getM01();
   }

   /**
    * Gets the 1st row 3rd column coefficient of this transform.
    *
    * @return the 1st row 3rd column coefficient.
    */
   public double getM02()
   {
      return linearTransform.getM02();
   }

   /**
    * Gets the 1st row 4th column coefficient of this transform.
    *
    * @return the 1st row 4th column coefficient.
    */
   public double getM03()
   {
      return translationVector.getX();
   }

   /**
    * Gets the 2nd row 1st column coefficient of this transform.
    *
    * @return the 2nd row 1st column coefficient.
    */
   public double getM10()
   {
      return linearTransform.getM10();
   }

   /**
    * Gets the 2nd row 2nd column coefficient of this transform.
    *
    * @return the 2nd row 2nd column coefficient.
    */
   public double getM11()
   {
      return linearTransform.getM11();
   }

   /**
    * Gets the 2nd row 3rd column coefficient of this transform.
    *
    * @return the 2nd row 3rd column coefficient.
    */
   public double getM12()
   {
      return linearTransform.getM12();
   }

   /**
    * Gets the 2nd row 4th column coefficient of this transform.
    *
    * @return the 2nd row 4th column coefficient.
    */
   public double getM13()
   {
      return translationVector.getY();
   }

   /**
    * Gets the 3rd row 1st column coefficient of this transform.
    *
    * @return the 3rd row 1st column coefficient.
    */
   public double getM20()
   {
      return linearTransform.getM20();
   }

   /**
    * Gets the 3rd row 2nd column coefficient of this transform.
    *
    * @return the 3rd row 2nd column coefficient.
    */
   public double getM21()
   {
      return linearTransform.getM21();
   }

   /**
    * Gets the 3rd row 3rd column coefficient of this transform.
    *
    * @return the 3rd row 3rd column coefficient.
    */
   public double getM22()
   {
      return linearTransform.getM22();
   }

   /**
    * Gets the 3rd row 4th column coefficient of this transform.
    *
    * @return the 3rd row 4th column coefficient.
    */
   public double getM23()
   {
      return translationVector.getZ();
   }

   /**
    * Gets the 4th row 1st column coefficient of this transform.
    * <p>
    * Note: {@code m30 = 0.0}.
    * </p>
    *
    * @return the 4th row 1st column coefficient.
    */
   public double getM30()
   {
      return 0.0;
   }

   /**
    * Gets the 4th row 2nd column coefficient of this transform.
    * <p>
    * Note: {@code m31 = 0.0}.
    * </p>
    *
    * @return the 4th row 2nd column coefficient.
    */
   public double getM31()
   {
      return 0.0;
   }

   /**
    * Gets the 4th row 3rd column coefficient of this transform.
    * <p>
    * Note: {@code m32 = 0.0}.
    * </p>
    *
    * @return the 4th row 3rd column coefficient.
    */
   public double getM32()
   {
      return 0.0;
   }

   /**
    * Gets the 4th row 4th column coefficient of this transform.
    * <p>
    * Note: {@code m33 = 1.0}.
    * </p>
    *
    * @return the 4th row 4th column coefficient.
    */
   public double getM33()
   {
      return 1.0;
   }

   /**
    * Tests separately and on a per component basis if the rotation part, the scale part, and the
    * translation part of this transform and {@code other} are equal to an {@code epsilon}.
    *
    * @param other the other affine transform to compare against this. Not modified.
    */
   @Override
   public boolean epsilonEquals(AffineTransform other, double epsilon)
   {
      return AffineTransformBasics.super.epsilonEquals(other, epsilon);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(AffineTransform)}, it returns {@code false} otherwise or if the {@code object} is
    * {@code null}.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof AffineTransformReadOnly)
         return equals((AffineTransformReadOnly) object);
      else
         return false;
   }

   /**
    * Two affine transforms are considered geometrically equal if both the rotation-scale matrices and
    * translation vectors are equal.
    *
    * @param other   the other affine transform to compare against this. Not modified.
    * @param epsilon the tolerance to use when comparing each component.
    * @return {@code true} if the two rigid body transforms are equal, {@code false} otherwise.
    */
   @Override
   public boolean geometricallyEquals(AffineTransform other, double epsilon)
   {
      return AffineTransformBasics.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Provides a {@code String} representation of this transform as follows: <br>
    * m00, m01, m02 | m03 <br>
    * m10, m11, m12 | m13 <br>
    * m20, m21, m22 | m23
    *
    * @return the {@code String} representing this transform.
    */
   @Override
   public String toString()
   {
      return EuclidCoreIOTools.getAffineTransformString(this);
   }

   @Override
   public int hashCode()
   {
      long bits = EuclidHashCodeTools.addToHashCode(linearTransform.hashCode(), translationVector.hashCode());
      return EuclidHashCodeTools.toIntHashCode(bits);
   }
}
