package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FramePointShape3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePointShape3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

/**
 * Implementation of a point shape 3D expressed in a given reference frame.
 * <p>
 * A point shape 3D is represented by its position.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class FramePointShape3D implements FramePointShape3DBasics, GeometryObject<FramePointShape3D>
{
   /** The reference frame in which this shape is expressed. */
   private ReferenceFrame referenceFrame;
   private double x, y, z;

   /**
    * Creates a new point shape located at the origin and initializes its reference frame to
    * {@link ReferenceFrame#getWorldFrame()}.
    */
   public FramePointShape3D()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   /**
    * Creates a new point shape located at the origin and initializes its reference frame.
    * 
    * @param referenceFrame this shape initial reference frame.
    */
   public FramePointShape3D(ReferenceFrame referenceFrame)
   {
      setToZero(referenceFrame);
   }

   /**
    * Creates a new point shape and initializes its position.
    *
    * @param referenceFrame this shape initial reference frame.
    * @param x              the value for the x-coordinate of this shape.
    * @param y              the value for the y-coordinate of this shape.
    * @param z              the value for the z-coordinate of this shape.
    */
   public FramePointShape3D(ReferenceFrame referenceFrame, double x, double y, double z)
   {
      setIncludingFrame(referenceFrame, x, y, z);
   }

   /**
    * Creates a new point shape and initializes its position.
    *
    * @param referenceFrame  this shape initial reference frame.
    * @param tuple3DReadOnly the position of the point shape. Not modified.
    */
   public FramePointShape3D(ReferenceFrame referenceFrame, Tuple3DReadOnly tuple3DReadOnly)
   {
      setIncludingFrame(referenceFrame, tuple3DReadOnly);
   }

   /**
    * Creates a new point shape and initializes its position.
    *
    * @param tuple3DReadOnly the position of the point shape. Not modified.
    */
   public FramePointShape3D(FrameTuple3DReadOnly tuple3DReadOnly)
   {
      setIncludingFrame(tuple3DReadOnly);
   }

   /** {@inheritDoc} */
   @Override
   public void set(FramePointShape3D other)
   {
      FramePointShape3DBasics.super.set(other);
   }

   /** {@inheritDoc} */
   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }

   /** {@inheritDoc} */
   @Override
   public void setX(double x)
   {
      this.x = x;
   }

   /** {@inheritDoc} */
   @Override
   public void setY(double y)
   {
      this.y = y;
   }

   /** {@inheritDoc} */
   @Override
   public void setZ(double z)
   {
      this.z = z;
   }

   /** {@inheritDoc} */
   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   /** {@inheritDoc} */
   @Override
   public double getX()
   {
      return x;
   }

   /** {@inheritDoc} */
   @Override
   public double getY()
   {
      return y;
   }

   /** {@inheritDoc} */
   @Override
   public double getZ()
   {
      return z;
   }

   /** {@inheritDoc} */
   @Override
   public FramePointShape3D copy()
   {
      return new FramePointShape3D(this);
   }

   /**
    * Tests on a per component basis if {@code other} and {@code this} are equal to an {@code epsilon}.
    * <p>
    * If the two point shapes have different frames, this method returns {@code false}.
    * </p>
    *
    * @param other   the other point shape to compare against this. Not modified.
    * @param epsilon tolerance to use when comparing each component.
    * @return {@code true} if the two point shapes are equal component-wise and are expressed in the
    *         same reference frame, {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(FramePointShape3D other, double epsilon)
   {
      return FramePointShape3DBasics.super.epsilonEquals(other, epsilon);
   }

   /**
    * Compares {@code this} and {@code other} to determine if the two point shapes are geometrically
    * similar.
    *
    * @param other   the point shape to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the point shapes represent the same geometry, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code other} are not expressed in
    *                                         the same reference frame.
    */
   @Override
   public boolean geometricallyEquals(FramePointShape3D other, double epsilon)
   {
      return FramePointShape3DBasics.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(FrameTuple3DReadOnly)}, it returns {@code false} otherwise.
    * <p>
    * If the two point shapes have different frames, this method returns {@code false}.
    * </p>
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal and are expressed in the same
    *         reference frame, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof FramePointShape3DReadOnly)
         return FramePointShape3DBasics.super.equals((FramePointShape3DReadOnly) object);
      else
         return false;
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this point shape 3D.
    *
    * @return the hash code value for this point shape 3D.
    */
   @Override
   public int hashCode()
   {
      long bits = EuclidHashCodeTools.toLongHashCode(x, y, z);
      bits = EuclidHashCodeTools.addToHashCode(bits, referenceFrame);
      return EuclidHashCodeTools.toIntHashCode(bits);
   }

   /**
    * Provides a {@code String} representation of this point shape 3D as follows:
    *
    * <pre>
    * Point shape 3D: (-0.362, -0.617,  0.066 ) - worldFrame
    * </pre>
    *
    * @return the {@code String} representing this point shape 3D.
    */
   @Override
   public String toString()
   {
      return EuclidFrameShapeIOTools.getFramePointShape3DString(this);
   }
}
