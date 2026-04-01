export type Point = { x: number; y: number };

export type Capture = {
  id: string;
  time: string;
  desc: string | null;
  direction: string | null;
  colour: string | null;
  reference: string | null;
  green: Point | null;
  red: Point | null;
  imageUrl: string | null;
};

export type ImagePair = {
  forwardUrl: string;
  downwardUrl: string;
  pk: string;
  oakdWidth: number;
  oakdHeight: number;
  arduWidth: number;
  arduHeight: number;
} | null;

export type PopupState = {
  url: string;
  imageType: "forward" | "downward";
  green: Point | null;
  red: Point | null;
} | null;

export type ForwardAnnotation = {
  target: Point | null;
  ref: Point | null;
} | null;

export type DownwardAnnotation = {
  target: Point | null;
} | null;
