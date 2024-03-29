package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.Axis2D;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.referenceFrame.interfaces.EuclidFrameGeometry;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameUnitVector2DBasics;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.UnitVector2D;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.UnitVector2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

/**
 * Implementation for a 2 dimensional unit-length vector expressed in a given reference frame.
 * <p>
 * This unit vector shares the same API as a regular vector 2D while ensuring it is normalized when
 * accessing directly or indirectly its individual components, i.e. when invoking either
 * {@link #getX()} or {@link #getY()}}.
 * </p>
 * <p>
 * When the values of this vector are set to zero, the next time it is normalized it will be reset
 * to (1.0, 0.0).
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class FrameUnitVector2D implements FrameUnitVector2DBasics, Settable<FrameUnitVector2D>
{
   /** The reference frame is which this vector is currently expressed. */
   private ReferenceFrame referenceFrame;
   /** The vector holding the current components of this frame vector. */
   private final UnitVector2D vector = new UnitVector2D();
   /** Rigid-body transform used to perform garbage-free operations. */
   private final RigidBodyTransform transformToDesiredFrame = new RigidBodyTransform();

   /**
    * Creates a new frame unit vector and initializes it to {@link Axis2D#X} and its reference frame to
    * {@link ReferenceFrame#getWorldFrame()}.
    */
   public FrameUnitVector2D()
   {
      setToZero(ReferenceFrame.getWorldFrame());
   }

   /**
    * Creates a new frame unit vector and initializes it to {@link Axis2D#X} and its reference frame to
    * the {@code referenceFrame}.
    *
    * @param referenceFrame the initial frame for this frame unit vector.
    */
   public FrameUnitVector2D(ReferenceFrame referenceFrame)
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
    */
   public FrameUnitVector2D(ReferenceFrame referenceFrame, double x, double y)
   {
      setIncludingFrame(referenceFrame, x, y);
   }

   /**
    * Creates a new frame unit vector and initializes its component {@code x}, {@code y} in order from
    * the given array and initializes its reference frame.
    *
    * @param referenceFrame the initial frame for this frame unit vector.
    * @param vectorArray    the array containing this unit vector's components. Not modified.
    */
   public FrameUnitVector2D(ReferenceFrame referenceFrame, double[] vectorArray)
   {
      setIncludingFrame(referenceFrame, vectorArray);
   }

   /**
    * Creates a new frame unit vector and initializes it to {@code tuple2DReadOnly} and to the given
    * reference frame.
    *
    * @param referenceFrame  the initial frame for this frame unit vector.
    * @param tuple2DReadOnly the tuple to copy the components from. Not modified.
    */
   public FrameUnitVector2D(ReferenceFrame referenceFrame, Tuple2DReadOnly tuple2DReadOnly)
   {
      setIncludingFrame(referenceFrame, tuple2DReadOnly);
   }

   /**
    * Creates a new frame unit vector and initializes its x and y coordinate to {@code tuple3DReadOnly}
    * and to the given reference frame.
    *
    * @param referenceFrame  the initial frame for this frame unit vector.
    * @param tuple3DReadOnly the tuple to copy the coordinates from. Not modified.
    */
   public FrameUnitVector2D(ReferenceFrame referenceFrame, Tuple3DReadOnly tuple3DReadOnly)
   {
      setIncludingFrame(referenceFrame, tuple3DReadOnly);
   }

   /**
    * Creates a new frame unit vector and initializes its reference frame x and y components from
    * {@code frameTuple3DReadOnly}.
    *
    * @param frameTuple3DReadOnly the tuple to copy the components and reference frame from. Not
    *                             modified.
    */
   public FrameUnitVector2D(FrameTuple3DReadOnly frameTuple3DReadOnly)
   {
      setIncludingFrame(frameTuple3DReadOnly);
   }

   /**
    * Creates a new frame unit vector and initializes it to {@code other}.
    *
    * @param other the tuple to copy the components and reference frame from. Not modified.
    */
   public FrameUnitVector2D(FrameTuple2DReadOnly other)
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
   public void set(FrameUnitVector2D other)
   {
      FrameUnitVector2DBasics.super.set(other);
   }

   /** {@inheritDoc} */
   @Override
   public void set(UnitVector2DReadOnly other)
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
   public void changeFrame(ReferenceFrame desiredFrame)
   {
      // Check for the trivial case: the geometry is already expressed in the desired frame.
      if (desiredFrame == referenceFrame)
         return;

      /*
       * By overriding changeFrame, on the transformToDesiredFrame is being checked instead of checking
       * both referenceFrame.transformToRoot and desiredFrame.transformToRoot.
       */
      referenceFrame.getTransformToDesiredFrame(transformToDesiredFrame, desiredFrame);
      applyTransform(transformToDesiredFrame);
      referenceFrame = desiredFrame;
   }

   /** {@inheritDoc} */
   @Override
   public final void changeFrameAndProjectToXYPlane(ReferenceFrame desiredFrame)
   {
      // Check for the trivial case: the geometry is already expressed in the desired frame.
      if (desiredFrame == referenceFrame)
         return;

      referenceFrame.getTransformToDesiredFrame(transformToDesiredFrame, desiredFrame);
      applyTransform(transformToDesiredFrame, false);
      referenceFrame = desiredFrame;
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(EuclidFrameGeometry)}, it returns {@code false} otherwise.
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
      if (object instanceof FrameTuple2DReadOnly)
         return equals((EuclidFrameGeometry) object);
      else
         return false;
   }

   /**
    * Provides a {@code String} representation of this frame vector 2D as follows: (x, y)-worldFrame.
    *
    * @return the {@code String} representing this frame vector 2D.
    */
   @Override
   public String toString()
   {
      return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this frame vector
    * 2D.
    *
    * @return the hash code value for this frame vector 2D.
    */
   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(vector, referenceFrame);
   }
}
