import React, {type ReactNode} from 'react';
import Layout from '@theme-original/Layout';
import type LayoutType from '@theme/Layout';
import type {WrapperProps} from '@docusaurus/types';
import { ChatWidget } from '../../components/Chatbot/ChatWidget'; // Import ChatWidget

type Props = WrapperProps<typeof LayoutType>;

export default function LayoutWrapper(props: Props): ReactNode {
  return (
    <>
      <Layout {...props} />
      <ChatWidget /> {/* Render the ChatWidget here */}
    </>
  );
}
