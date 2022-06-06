package us.ihmc.euclid.transform;

import org.ejml.data.DMatrix;

import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.matrix.LinearTransform3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tools.Matrix3DTools;
import us.ihmc.euclid.transform.interfaces.AffineTransformBasics;
import us.ihmc.euclid.transform.interfaces.AffineTransformReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

/**
 * A {@code AffineTransform} represents a 4-by-4 transformation matrix that can rotate, scale,
 * shear, and translate.
 * <p>
 * The {@code AffineTransform} is composed of {@link LinearTransform3D} to rotate, scale, shear, and
 * a {@link Vector3D} to translate.
 * </p>
 * <p>
 * A few special cases to keep in mind:
 * <ul>
 * <li>when transforming a {@link Orientation3DBasics}, only the rotation part of the
 * {@link LinearTransform3D} is prepended to the quaternion, such that the output remains a proper
 * unit-quaternion that still only describes a rotation.
 * <li>when applying this transform on a {@link Point3DBasics} or {@link Point2DBasics}, this object
 * is, in order, transformed with the {@link LinearTransform3D} and then translated.
 * <li>when applying this transform on a {@link Vector3DBasics} or {@link Vector2DBasics}, this
 * object is transformed with the {@link LinearTransform3D}. It is NOT translated.
 * </ul>
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public class AffineTransform
      implements AffineTransformBasics, Settable<AffineTransform>
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
      setIdentity();
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
    *
    * @param rigidBodyTransform the rigid-body transform to copy. Not modified.
    */
   public AffineTransform(RigidBodyTransformReadOnly rigidBodyTransform)
   {
      set(rigidBodyTransform);
   }

   /**
    * Creates a new affine transform and initializes it from the given 3D matrix and the given
    * translation.
    *
    * @param linearTransform the matrix used to initialize the linear part of the transform. Not
    *                        modified.
    * @param translation     the translation to copy. Not modified.
    */
   public AffineTransform(Matrix3DReadOnly linearTransform, Tuple3DReadOnly translation)
   {
      set(linearTransform, translation);
   }

   /**
    * Creates a new affine transform and initializes it from the given 3D matrix and the given
    * translation.
    *
    * @param linearTransform the matrix used to initialize the linear part of the transform. Not
    *                        modified.
    * @param translation     the translation to copy. Not modified.
    */
   public AffineTransform(DMatrix linearTransform, Tuple3DReadOnly translation)
   {
      set(linearTransform, translation);
   }

   /**
    * Creates a new affine transform and initializes it from the given rotation matrix and the given
    * translation.
    *
    * @param rotationMatrix the matrix used to initialize the linear part of the transform. Not
    *                       modified.
    * @param translation    the translation to copy. Not modified.
    */
   public AffineTransform(RotationMatrix rotationMatrix, Tuple3DReadOnly translation)
   {
      set(rotationMatrix, translation);
   }

   /**
    * Creates a new affine transform and initializes it from the given orientation and the given
    * translation.
    *
    * @param orientation the orientation used to initialize the linear part of the transform. Not
    *                    modified.
    * @param translation the translation to copy. Not modified.
    */
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

   /**
    * Returns the read-only view of the linear part of this transform as a pure orientation.
    * <p>
    * The orientation represents the same transformation as {@link #getLinearTransform()} with the
    * scales set to 1. The returned quaternion is linked to this transform, i.e. it is automatically
    * updated when this transform is modified.
    * </p>
    * 
    * @return the read-only view of the linear part of this transform as a pure orientation.
    */
   public QuaternionReadOnly getRotationView()
   {
      return linearTransform.getAsQuaternion();
   }

   /**
    * Gets the reference to the linear part of this transform.
    *
    * @return the linear part of this transform.
    */
   @Override
   public LinearTransform3D getLinearTransform()
   {
      return linearTransform;
   }

   /**
    * Gets the reference of the translation part of this affine transform.
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
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(AffineTransformReadOnly)}, it returns {@code false} otherwise or if the
    * {@code object} is {@code null}.
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
    * Provides a {@code String} representation of this transform as follows:
    * 
    * <pre>
    *  0.596  0.630  0.930 | -0.435
    * -0.264  0.763  0.575 | -0.464
    * -0.430 -0.188 -0.048 |  0.611
    *  0.000  0.000  0.000 |  1.000
    * </pre>
    *
    * @return the {@code String} representing this transform.
    */
   @Override
   public String toString()
   {
      return AffineTransformBasics.super.toString(null);
   }

   @Override
   public int hashCode()
   {
      long bits = EuclidHashCodeTools.addToHashCode(linearTransform.hashCode(), translationVector.hashCode());
      return EuclidHashCodeTools.toIntHashCode(bits);
   }
}
