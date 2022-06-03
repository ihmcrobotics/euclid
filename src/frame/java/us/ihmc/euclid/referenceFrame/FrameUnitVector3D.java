package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameUnitVector3DBasics;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.UnitVector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DReadOnly;

/**
 * Implementation for a 3 dimensional unit-length vector expressed in a given reference frame.
 * <p>
 * This unit vector shares the same API as a regular vector 3D while ensuring it is normalized when
 * accessing directly or indirectly its individual components, i.e. when invoking either
 * {@link #getX()}, {@link #getY()}, or {@link #getZ()}.
 * </p>
 * <p>
 * When the values of this vector are set to zero, the next time it is normalized it will be reset
 * to (1.0, 0.0, 0.0).
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class FrameUnitVector3D implements FrameUnitVector3DBasics
{
   /** The reference frame is which this vector is currently expressed. */
   private ReferenceFrame referenceFrame;
   /** The vector holding the current components of this frame vector. */
   private final UnitVector3D vector = new UnitVector3D();

   /**
    * Creates a new frame unit vector and initializes it to {@link Axis3D#X} and its reference frame to
    * {@link ReferenceFrame#getWorldFrame()}.
    */
   public FrameUnitVector3D()
   {
      setToZero(ReferenceFrame.getWorldFrame());
   }

   /**
    * Creates a new frame unit vector and initializes it to {@link Axis3D#X} and its reference frame to
    * the {@code referenceFrame}.
    *
    * @param referenceFrame the initial frame for this frame unit vector.
    */
   public FrameUnitVector3D(ReferenceFrame referenceFrame)
   {
      setToZero(referenceFrame);
   }

   /**
    * Creates a new frame unit vector and initializes it with the given components and the given
    * reference frame.
    *
    * @param referenceFrame the initial frame for this frame unit vector.
    * @param x              the x-component.
    * @param y              the y-component.
    * @param z              the z-component.
    */
   public FrameUnitVector3D(ReferenceFrame referenceFrame, double x, double y, double z)
   {
      setIncludingFrame(referenceFrame, x, y, z);
   }

   /**
    * Creates a new frame unit vector and initializes its component {@code x}, {@code y}, {@code z} in
    * order from the given array and initializes its reference frame.
    *
    * @param referenceFrame the initial frame for this frame unit vector.
    * @param vectorArray    the array containing this unit vector's components. Not modified.
    */
   public FrameUnitVector3D(ReferenceFrame referenceFrame, double[] vectorArray)
   {
      setIncludingFrame(referenceFrame, vectorArray);
   }

   /**
    * Creates a new frame unit vector and initializes it to {@code tuple3DReadOnly} and to the given
    * reference frame.
    *
    * @param referenceFrame  the initial frame for this frame unit vector.
    * @param tuple3DReadOnly the tuple to copy the components from. Not modified.
    */
   public FrameUnitVector3D(ReferenceFrame referenceFrame, Tuple3DReadOnly tuple3DReadOnly)
   {
      setIncludingFrame(referenceFrame, tuple3DReadOnly);
   }

   /**
    * Creates a new frame unit vector and initializes its x and y coordinate to {@code tuple2DReadOnly}
    * and to the given reference frame.
    *
    * @param referenceFrame  the initial frame for this frame unit vector.
    * @param tuple2DReadOnly the tuple to copy the coordinates from. Not modified.
    */
   public FrameUnitVector3D(ReferenceFrame referenceFrame, Tuple2DReadOnly tuple2DReadOnly)
   {
      setIncludingFrame(referenceFrame, tuple2DReadOnly, 0.0);
   }

   /**
    * Creates a new frame unit vector and initializes its reference frame x and y components from
    * {@code frameTuple2DReadOnly}.
    *
    * @param frameTuple2DReadOnly the tuple to copy the components and reference frame from. Not
    *                             modified.
    */
   public FrameUnitVector3D(FrameTuple2DReadOnly frameTuple2DReadOnly)
   {
      setIncludingFrame(frameTuple2DReadOnly, 0.0);
   }

   /**
    * Creates a new frame unit vector and initializes it to {@code other}.
    *
    * @param other the tuple to copy the components and reference frame from. Not modified.
    */
   public FrameUnitVector3D(FrameTuple3DReadOnly other)
   {
      setIncludingFrame(other);
   }

   /** {@inheritDoc} */
   @Override
   public void absolute()
   {
      vector.absolute();
   }

   /** {@inheritDoc} */
   @Override
   public void negate()
   {
      vector.negate();
   }

   /** {@inheritDoc} */
   @Override
   public void normalize()
   {
      vector.normalize();
   }

   /** {@inheritDoc} */
   @Override
   public void markAsDirty()
   {
      vector.markAsDirty();
   }

   /** {@inheritDoc} */
   @Override
   public boolean isDirty()
   {
      return vector.isDirty();
   }

   /** {@inheritDoc} */
   @Override
   public void set(UnitVector3DReadOnly other)
   {
      vector.set(other);
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
      vector.setX(x);
   }

   /** {@inheritDoc} */
   @Override
   public void setY(double y)
   {
      vector.setY(y);
   }

   /** {@inheritDoc} */
   @Override
   public void setZ(double z)
   {
      vector.setZ(z);
   }

   /** {@inheritDoc} */
   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   /** {@inheritDoc} */
   @Override
   public double getRawX()
   {
      return vector.getRawX();
   }

   /** {@inheritDoc} */
   @Override
   public double getRawY()
   {
      return vector.getRawY();
   }

   /** {@inheritDoc} */
   @Override
   public double getRawZ()
   {
      return vector.getRawZ();
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(FrameTuple3DReadOnly)}, it returns {@code false} otherwise.
    * <p>
    * If the two vectors have different frames, this method returns {@code false}.
    * </p>
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if the two vectors are exactly equal component-wise and are expressed in the
    *         same reference frame, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof FrameTuple3DReadOnly)
         return equals((FrameTuple3DReadOnly) object);
      else
         return false;
   }

   /**
    * Provides a {@code String} representation of this frame vector 3D as follows: (x, y,
    * z)-worldFrame.
    *
    * @return the {@code String} representing this frame vector 3D.
    */
   @Override
   public String toString()
   {
      return EuclidFrameIOTools.getFrameTuple3DString(this);
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this frame vector
    * 3D.
    *
    * @return the hash code value for this frame vector 3D.
    */
   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(vector, referenceFrame);
   }
}
