export type Point = { x: number; y: number };

export type Capture = {
  id: string;
  time: string;
  colour: string | null;
  direction: string | null;
  reference: string | null;
  desc: string | null;
  imageUrl: string | null;
  green: Point | null;
  red: Point | null;
};

export type ImagePair = { forwardUrl: string; downwardUrl: string } | null;

export type PopupState = {
  url: string;
  imageType: "forward" | "downward";
  green: Point | null;
  red: Point | null;
} | null;

export type SavedAnnotation = {
  green: Point | null;
  red: Point | null;
  imageType: "forward" | "downward" | null;
} | null;
