package us.ihmc.euclid.geometry.exceptions;

public class EmptyPolygonException extends RuntimeException
{
   private static final long serialVersionUID = -323833885395952453L;

   public EmptyPolygonException(String description)
   {
      super(description);
   }
}