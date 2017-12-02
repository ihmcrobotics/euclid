package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.geometry.Orientation2D;
import us.ihmc.euclid.geometry.interfaces.Orientation2DBasics;
import us.ihmc.euclid.geometry.interfaces.Orientation2DReadOnly;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation2DReadOnly;

public class FrameOrientation2D extends FrameGeometryObject<FrameOrientation2D, Orientation2D> implements FrameOrientation2DReadOnly, Orientation2DBasics
{
   private final Orientation2D orientation;

   public FrameOrientation2D(Orientation2D orientation)
   {
      super(orientation);
      this.orientation = getGeometryObject();
   }

   public FrameOrientation2D(ReferenceFrame referenceFrame, Orientation2DReadOnly orientation)
   {
      super(referenceFrame, new Orientation2D(orientation));
      this.orientation = getGeometryObject();
   }

   @Override
   public void setYaw(double yaw)
   {
      orientation.setYaw(yaw);
   }

   @Override
   public double getYaw()
   {
      return orientation.getYaw();
   }

   /**
    * Sets this orientation 2D to the {@code other} orientation 2D.
    *
    * @param other the other orientation 2D. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *            {@code this}.
    */
   public void set(FrameOrientation2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      
      Orientation2DBasics.super.set(other);
   }

   /**
    * Adds the other orientation 2D to this:<br>
    * {@code this += other}
    * <p>
    * Note that resulting angle is computed such that it is contained in [-<i>pi</i>, <i>pi</pi>].
    * </p>
    *
    * @param other the other orientation 2D to add to this. Not modified.
    */
   public void add(FrameOrientation2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      
      Orientation2DBasics.super.add(other);
   }

   /**
    * Sets this orientation 2D to the sum of the two given orientation 2Ds:<br>
    * {@code this = orientation1 + orientation2}
    * <p>
    * Note that resulting angle is computed such that it is contained in [-<i>pi</i>, <i>pi</pi>].
    * </p>
    *
    * @param orientation1 the first orientation 2D. Not modified.
    * @param orientation2 the second orientation 2D. Not modified.
    */
   public void add(FrameOrientation2DReadOnly orientation1, Orientation2DReadOnly orientation2)
   {
      checkReferenceFrameMatch(orientation1);
      
      Orientation2DBasics.super.add(orientation1, orientation2);
   }

   /**
    * Sets this orientation 2D to the sum of the two given orientation 2Ds:<br>
    * {@code this = orientation1 + orientation2}
    * <p>
    * Note that resulting angle is computed such that it is contained in [-<i>pi</i>, <i>pi</pi>].
    * </p>
    *
    * @param orientation1 the first orientation 2D. Not modified.
    * @param orientation2 the second orientation 2D. Not modified.
    */
   public void add(Orientation2DReadOnly orientation1, FrameOrientation2DReadOnly orientation2)
   {
      checkReferenceFrameMatch(orientation2);
      
      Orientation2DBasics.super.add(orientation1, orientation2);
   }

   /**
    * Sets this orientation 2D to the sum of the two given orientation 2Ds:<br>
    * {@code this = orientation1 + orientation2}
    * <p>
    * Note that resulting angle is computed such that it is contained in [-<i>pi</i>, <i>pi</pi>].
    * </p>
    *
    * @param orientation1 the first orientation 2D. Not modified.
    * @param orientation2 the second orientation 2D. Not modified.
    */
   public void add(FrameOrientation2DReadOnly orientation1, FrameOrientation2DReadOnly orientation2)
   {
      checkReferenceFrameMatch(orientation1);
      checkReferenceFrameMatch(orientation2);
      
      Orientation2DBasics.super.add(orientation1, orientation2);
   }
   
   /**
    * Subtracts the other orientation 2D to this:<br>
    * {@code this -= other}
    * <p>
    * Note that resulting angle is computed such that it is contained in [-<i>pi</i>, <i>pi</pi>].
    * </p>
    *
    * @param other the other orientation 2D to add to this. Not modified.
    */
   public void sub(FrameOrientation2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      
      Orientation2DBasics.super.sub(other);
   }

   /**
    * Sets this orientation 2D to the difference of the two given orientation 2Ds:<br>
    * {@code this = orientation1 - orientation2}
    * <p>
    * Note that resulting angle is computed such that it is contained in [-<i>pi</i>, <i>pi</pi>].
    * </p>
    *
    * @param orientation1 the first orientation 2D. Not modified.
    * @param orientation2 the second orientation 2D. Not modified.
    */
   public void sub(FrameOrientation2DReadOnly orientation1, Orientation2DReadOnly orientation2)
   {
      checkReferenceFrameMatch(orientation1);
      
      Orientation2DBasics.super.sub(orientation1, orientation2);
   }

   /**
    * Sets this orientation 2D to the difference of the two given orientation 2Ds:<br>
    * {@code this = orientation1 - orientation2}
    * <p>
    * Note that resulting angle is computed such that it is contained in [-<i>pi</i>, <i>pi</pi>].
    * </p>
    *
    * @param orientation1 the first orientation 2D. Not modified.
    * @param orientation2 the second orientation 2D. Not modified.
    */
   public void sub(Orientation2DReadOnly orientation1, FrameOrientation2DReadOnly orientation2)
   {
      checkReferenceFrameMatch(orientation2);
      
      Orientation2DBasics.super.sub(orientation1, orientation2);
   }

   /**
    * Sets this orientation 2D to the difference of the two given orientation 2Ds:<br>
    * {@code this = orientation1 - orientation2}
    * <p>
    * Note that resulting angle is computed such that it is contained in [-<i>pi</i>, <i>pi</pi>].
    * </p>
    *
    * @param orientation1 the first orientation 2D. Not modified.
    * @param orientation2 the second orientation 2D. Not modified.
    */
   public void sub(FrameOrientation2DReadOnly orientation1, FrameOrientation2DReadOnly orientation2)
   {
      checkReferenceFrameMatch(orientation1);
      checkReferenceFrameMatch(orientation2);
      
      Orientation2DBasics.super.sub(orientation1, orientation2);
   }

   /**
    * Performs a linear interpolation from {@code this} to {@code other} given the percentage
    * {@code alpha}.
    * <p>
    * this = (1.0 - alpha) * this + alpha * other
    * </p>
    *
    * @param other the other orientation 2D used for the interpolation. Not modified.
    * @param alpha the percentage used for the interpolation. A value of 0 will result in not
    *           modifying {@code this}, while a value of 1 is equivalent to setting {@code this} to
    *           {@code other}.
    */
   public void interpolate(FrameOrientation2DReadOnly other, double alpha)
   {
      checkReferenceFrameMatch(other);
      
      Orientation2DBasics.super.interpolate(other, alpha);
   }

   /**
    * Performs a linear interpolation from {@code orientation1} to {@code orientation2} given the
    * percentage {@code alpha}.
    * <p>
    * this = (1.0 - alpha) * orientation1 + alpha * orientation2
    * </p>
    *
    * @param orientation1 the first orientation 2D used in the interpolation. Not modified.
    * @param orientation2 the second orientation 2D used in the interpolation. Not modified.
    * @param alpha the percentage to use for the interpolation. A value of 0 will result in setting
    *           {@code this} to {@code orientation1}, while a value of 1 is equivalent to setting
    *           {@code this} to {@code orientation2}.
    */
   public void interpolate(FrameOrientation2DReadOnly orientation1, Orientation2DReadOnly orientation2, double alpha)
   {
      checkReferenceFrameMatch(orientation1);
      
      Orientation2DBasics.super.interpolate(orientation1, orientation2, alpha);
   }

   /**
    * Performs a linear interpolation from {@code orientation1} to {@code orientation2} given the
    * percentage {@code alpha}.
    * <p>
    * this = (1.0 - alpha) * orientation1 + alpha * orientation2
    * </p>
    *
    * @param orientation1 the first orientation 2D used in the interpolation. Not modified.
    * @param orientation2 the second orientation 2D used in the interpolation. Not modified.
    * @param alpha the percentage to use for the interpolation. A value of 0 will result in setting
    *           {@code this} to {@code orientation1}, while a value of 1 is equivalent to setting
    *           {@code this} to {@code orientation2}.
    */
   public void interpolate(Orientation2DReadOnly orientation1, FrameOrientation2DReadOnly orientation2, double alpha)
   {
      checkReferenceFrameMatch(orientation2);
      
      Orientation2DBasics.super.interpolate(orientation1, orientation2, alpha);
   }

   /**
    * Performs a linear interpolation from {@code orientation1} to {@code orientation2} given the
    * percentage {@code alpha}.
    * <p>
    * this = (1.0 - alpha) * orientation1 + alpha * orientation2
    * </p>
    *
    * @param orientation1 the first orientation 2D used in the interpolation. Not modified.
    * @param orientation2 the second orientation 2D used in the interpolation. Not modified.
    * @param alpha the percentage to use for the interpolation. A value of 0 will result in setting
    *           {@code this} to {@code orientation1}, while a value of 1 is equivalent to setting
    *           {@code this} to {@code orientation2}.
    */
   public void interpolate(FrameOrientation2DReadOnly orientation1, FrameOrientation2DReadOnly orientation2, double alpha)
   {
      checkReferenceFrameMatch(orientation1);
      checkReferenceFrameMatch(orientation2);
      
      Orientation2DBasics.super.interpolate(orientation1, orientation2, alpha);
   }
}
