package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.interfaces.Line2DBasics;
import us.ihmc.euclid.geometry.interfaces.Line2DReadOnly;
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
   
   @Override
   public void setDirection(double lineDirectionX, double lineDirectionY)
   {
      line.setDirection(lineDirectionX, lineDirectionY);
   }

   @Override
   public void setPoint(double pointOnLineX, double pointOnLineY)
   {
      line.setPoint(pointOnLineX, pointOnLineY);
   }

   @Override
   public void setDirectionUnsafe(double lineDirectionX, double lineDirectionY)
   {
      line.setDirectionUnsafe(lineDirectionX, lineDirectionY);
   }

   @Override
   public FramePoint2DReadOnly getPoint()
   {
      return new FramePoint2D(getReferenceFrame(), line.getPoint());
   }

   @Override
   public FrameVector2DReadOnly getDirection()
   {
      return new FrameVector2D(getReferenceFrame(), line.getDirection());
   }

   @Override
   public boolean hasPointBeenSet()
   {
      return line.hasPointBeenSet();
   }

   @Override
   public boolean hasDirectionBeenSet()
   {
      return line.hasDirectionBeenSet();
   }
}
