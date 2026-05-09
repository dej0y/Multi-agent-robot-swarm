import axios from 'axios';

const assignBot = async (objectName: string) => {
  try {
    const response = await axios.post('http://127.0.0.1:5000/assign', {
      object_name: objectName,
    });

    const { message, message_final } = response.data;
    return { success: true, messages: [message, message_final] };
  } catch (error: any) {
    return {
      success: false,
      messages: [error.response?.data?.message || "An error occurred"],
    };
  }
};

export default assignBot;
