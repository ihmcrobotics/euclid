package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.interfaces.Line2DBasics;
import us.ihmc.euclid.geometry.interfaces.Line2DReadOnly;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector2DReadOnly;

public class FrameLine2D extends FrameGeometryObject<FrameLine2D, Line2D> implements FrameLine2DReadOnly, Line2DBasics
{
   /** The line. */
   private final Line2D line;
   
   public FrameLine2D(Line2DReadOnly line2D)
   {
      this(ReferenceFrame.getWorldFrame(), line2D);
   }
   
   public FrameLine2D(ReferenceFrame referenceFrame, Line2DReadOnly line2D)
   {
      super(referenceFrame, new Line2D(line2D));
      line = getGeometryObject();
   }

   /** {@inheritDoc} */
   @Override
   public void setDirection(double lineDirectionX, double lineDirectionY)
   {
      line.setDirection(lineDirectionX, lineDirectionY);
   }
   
   /** {@inheritDoc} */
   @Override
   public void setPoint(double pointOnLineX, double pointOnLineY)
   {
      line.setPoint(pointOnLineX, pointOnLineY);
   }

   /**
    * Changes the point through which this line has to go.
    *
    * @param pointOnLine new point on this line. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code pointOnLine} are not expressed
    *          in the same reference frame.
    */
   public void setPoint(FramePoint2DReadOnly pointOnLine)
   {
      checkReferenceFrameMatch(pointOnLine);
      Line2DBasics.super.setPoint(pointOnLine);
   }

   /**
    * Changes the direction of this line by setting it to the normalized value of the given vector.
    *
    * @param lineDirection new direction of this line. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code lineDirection} are not expressed
    *          in the same reference frame.
    */
   public void setDirection(FrameVector2DReadOnly lineDirection)
   {
      checkReferenceFrameMatch(lineDirection);
      Line2DBasics.super.setDirection(lineDirection);
   }
   
   /** {@inheritDoc} */
   @Override
   public void setDirectionUnsafe(double lineDirectionX, double lineDirectionY)
   {
      line.setDirectionUnsafe(lineDirectionX, lineDirectionY);
   }

   /** {@inheritDoc} */
   @Override
   public FramePoint2DReadOnly getPoint()
   {
      return new FramePoint2D(getReferenceFrame(), line.getPoint());
   }

   /** {@inheritDoc} */
   @Override
   public FrameVector2DReadOnly getDirection()
   {
      return new FrameVector2D(getReferenceFrame(), line.getDirection());
   }
}
