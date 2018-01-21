package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.interfaces.Line3DBasics;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;

public class FrameLine3D extends FrameGeometryObject<FrameLine3D, Line3D> implements FrameLine3DReadOnly, Line3DBasics
{
   /** The line. */
   private final Line3D line;

   public FrameLine3D(Line3DReadOnly line3D)
   {
      this(ReferenceFrame.getWorldFrame(), line3D);
   }

   public FrameLine3D(ReferenceFrame referenceFrame, Line3DReadOnly line3D)
   {
      super(referenceFrame, new Line3D(line3D));
      line = getGeometryObject();
   }

   /** {@inheritDoc} */
   @Override
   public FramePoint3DReadOnly getPoint()
   {
      return new FramePoint3D(getReferenceFrame(), line.getPoint());
   }

   /** {@inheritDoc} */
   @Override
   public FrameVector3DReadOnly getDirection()
   {
      return new FrameVector3D(getReferenceFrame(), line.getDirection());
   }

   /** {@inheritDoc} */
   @Override
   public void setPoint(double pointOnLineX, double pointOnLineY, double pointOnLineZ)
   {
      line.setPoint(pointOnLineX, pointOnLineY, pointOnLineZ);
   }

   /** {@inheritDoc} */
   @Override
   public void setDirection(double lineDirectionX, double lineDirectionY, double lineDirectionZ)
   {
      line.setDirection(lineDirectionX, lineDirectionY, lineDirectionZ);
   }

   /** {@inheritDoc} */
   @Override
   public void setDirectionUnsafe(double lineDirectionX, double lineDirectionY, double lineDirectionZ)
   {
      line.setDirectionUnsafe(lineDirectionX, lineDirectionY, lineDirectionZ);
   }

   /**
    * Changes the point through which this line has to go.
    *
    * @param pointOnLine new point on this line. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code pointOnLine} are not expressed
    *          in the same reference frame.
    */
   public void setPoint(FramePoint3DReadOnly pointOnLine)
   {
      checkReferenceFrameMatch(pointOnLine);
      Line3DBasics.super.setPoint(pointOnLine);
   }

   /**
    * Changes the direction of this line by setting it to the normalized value of the given vector.
    *
    * @param lineDirection new direction of this line. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code lineDirection} are not expressed
    *          in the same reference frame.
    */
   public void setDirection(FrameVector3DReadOnly lineDirection)
   {
      checkReferenceFrameMatch(lineDirection);
      Line3DBasics.super.setDirection(lineDirection);
   }
}
